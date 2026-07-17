"""Read-only active polling for Voltron the Inquisitor."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Callable

from mili_voltron_defs import SOF


def build_packet(src: int, dst: int, cmd: int, index: int, payload: bytes = b"") -> bytes:
    """Build one framed packet using the confirmed additive-complement checksum."""

    if len(payload) > 255:
        raise ValueError("payload too long")
    body = bytes((len(payload), src, dst, cmd, index)) + payload
    checksum = 0xFFFF - (sum(body) & 0xFFFF)
    return SOF + body + checksum.to_bytes(2, "little")


def build_read_request(dst: int, index: int, requested_bytes: int) -> bytes:
    if not 0 <= requested_bytes <= 255:
        raise ValueError("requested byte count must fit in u8")
    return build_packet(0x3D, dst, 0x01, index, bytes((requested_bytes,)))


@dataclass(frozen=True, slots=True)
class PollRequest:
    dst: int
    index: int
    length: int
    name: str

    def frame(self) -> bytes:
        return build_read_request(self.dst, self.index, self.length)


@dataclass(slots=True)
class PendingRequest:
    request: PollRequest
    sent_at: float
    attempts: int


class InquisitorPoller:
    """Conservative single-flight scheduler for known read-only requests.

    Identity is treated specially. Register 0x10 is requested as a 16-byte
    block: the first 14 bytes are the BMS serial and bytes 14–15 are the BMS
    firmware word. If BMS communication is lost, identity becomes invalid and
    is requested again immediately after the first successful BMS reply.
    """

    PERIODIC_BMS = (
        PollRequest(0x22, 0x30, 12, "BMS_STATUS"),
        PollRequest(0x22, 0x40, 20, "BMS_CELLS"),
        PollRequest(0x22, 0x51, 6, "BMS_TEMPERATURES"),
    )
    IDENTITY_BMS = (
        PollRequest(0x22, 0x10, 16, "BMS_IDENTITY"),
        PollRequest(0x22, 0x3B, 2, "BMS_HEALTH"),
    )
    STARTUP_BMS = IDENTITY_BMS

    def __init__(
        self,
        *,
        poll_interval_s: float = 3.0,
        response_timeout_s: float = 0.35,
        inter_request_gap_s: float = 0.01,
        retries: int = 1,
        startup_identity: bool = True,
    ) -> None:
        if poll_interval_s <= 0:
            raise ValueError("poll interval must be positive")
        self.poll_interval_s = poll_interval_s
        self.response_timeout_s = response_timeout_s
        self.inter_request_gap_s = inter_request_gap_s
        self.retries = max(0, retries)

        self.queue: deque[PollRequest] = deque(self.IDENTITY_BMS if startup_identity else ())
        self.pending: PendingRequest | None = None
        self.next_cycle_at = 0.0
        self.next_send_at = 0.0
        self.started = False

        self.identity_required = startup_identity
        self.identity_valid = False
        self.bms_link_lost = False

        self.stats = {
            "sent": 0,
            "replies": 0,
            "timeouts": 0,
            "retries": 0,
            "unexpected_replies": 0,
            "link_losses": 0,
            "identity_refreshes": 0,
        }
        self.last_tx_name: str | None = None
        self.last_reply_name: str | None = None

    def start(self, now: float) -> None:
        self.started = True
        self.next_cycle_at = now
        self.next_send_at = now

    def _identity_is_queued(self) -> bool:
        identity_names = {request.name for request in self.IDENTITY_BMS}
        if self.pending is not None and self.pending.request.name in identity_names:
            return True
        return any(request.name in identity_names for request in self.queue)

    def request_identity(self, *, front: bool = True) -> None:
        """Queue one identity refresh, suppressing duplicate requests."""

        self.identity_required = True
        if self._identity_is_queued():
            return

        if front:
            for request in reversed(self.IDENTITY_BMS):
                self.queue.appendleft(request)
        else:
            self.queue.extend(self.IDENTITY_BMS)
        self.stats["identity_refreshes"] += 1

    def mark_bms_link_lost(self) -> None:
        """Invalidate identity; the first recovered BMS reply triggers refresh."""

        if not self.bms_link_lost:
            self.stats["link_losses"] += 1
        self.bms_link_lost = True
        self.identity_valid = False
        self.identity_required = True

    def _enqueue_periodic_if_due(self, now: float) -> None:
        if now < self.next_cycle_at or self.queue or self.pending is not None:
            return
        self.queue.extend(self.PERIODIC_BMS)
        # Avoid drift after pauses while also avoiding a catch-up packet storm.
        self.next_cycle_at = now + self.poll_interval_s

    def tick(self, now: float, send: Callable[[bytes], None]) -> None:
        if not self.started:
            self.start(now)

        if self.pending is not None:
            age = now - self.pending.sent_at
            if age >= self.response_timeout_s:
                if self.pending.attempts <= self.retries:
                    send(self.pending.request.frame())
                    self.pending.sent_at = now
                    self.pending.attempts += 1
                    self.stats["sent"] += 1
                    self.stats["retries"] += 1
                    self.last_tx_name = self.pending.request.name
                else:
                    self.stats["timeouts"] += 1
                    self.pending = None
                    self.next_send_at = now + self.inter_request_gap_s
            return

        self._enqueue_periodic_if_due(now)
        if self.queue and now >= self.next_send_at:
            request = self.queue.popleft()
            send(request.frame())
            self.pending = PendingRequest(request=request, sent_at=now, attempts=1)
            self.stats["sent"] += 1
            self.last_tx_name = request.name

    def observe(self, *, src: int, dst: int, cmd: int, index: int, now: float) -> bool:
        """Match an incoming reply. Returns True when it completes the pending poll."""

        if cmd != 0x04 or dst != 0x3D:
            return False
        pending = self.pending
        if pending is None:
            self.stats["unexpected_replies"] += 1
            return False

        req = pending.request
        if src != req.dst or index != req.index:
            self.stats["unexpected_replies"] += 1
            return False

        self.stats["replies"] += 1
        self.last_reply_name = req.name
        self.pending = None
        self.next_send_at = now + self.inter_request_gap_s

        recovered = self.bms_link_lost
        self.bms_link_lost = False

        if req.name == "BMS_IDENTITY":
            self.identity_valid = True
            self.identity_required = False
        elif recovered or self.identity_required:
            # Put identity ahead of the remaining CELL/TEMP requests. This
            # prevents a coherent CSV sample from completing under a stale or
            # missing battery identity after reconnection.
            self.request_identity(front=True)

        return True

    def snapshot(self, now: float) -> dict[str, object]:
        pending_age = None if self.pending is None else now - self.pending.sent_at
        return {
            **self.stats,
            "pending": None if self.pending is None else self.pending.request.name,
            "pending_age_s": pending_age,
            "queued": len(self.queue),
            "last_tx": self.last_tx_name,
            "last_reply": self.last_reply_name,
            "poll_interval_s": self.poll_interval_s,
            "identity_valid": self.identity_valid,
            "identity_required": self.identity_required,
            "bms_link_lost": self.bms_link_lost,
        }
