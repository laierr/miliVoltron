#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

has_live_source=false
has_replay_source=false
for arg in "$@"; do
    case "$arg" in
        -p|--port|--port=*) has_live_source=true ;;
        --binary|--binary=*|--hex|--hex=*|-h|--help|--list-ports) has_replay_source=true ;;
    esac
done

if [[ "$has_live_source" == false && "$has_replay_source" == false ]]; then
    declare -a candidates=()
    declare -A seen=()

    add_candidate() {
        local candidate="$1" target
        [[ -e "$candidate" ]] || return 0
        target="$(readlink -f -- "$candidate" 2>/dev/null || printf '%s' "$candidate")"
        [[ -n "${seen[$target]:-}" ]] && return 0
        seen[$target]=1
        candidates+=("$candidate")
    }

    shopt -s nullglob
    for port in /dev/serial/by-id/*; do add_candidate "$port"; done
    for port in /dev/ttyUSB* /dev/ttyACM*; do add_candidate "$port"; done
    shopt -u nullglob

    if (( ${#candidates[@]} == 0 )); then
        printf 'No serial device found under /dev/serial/by-id, /dev/ttyUSB*, or /dev/ttyACM*.\n' >&2
        exit 1
    elif (( ${#candidates[@]} == 1 )); then
        selected="${candidates[0]}"
    elif [[ -t 0 ]]; then
        printf 'Available serial devices:\n' >&2
        for index in "${!candidates[@]}"; do
            printf '  %d) %s\n' "$((index + 1))" "${candidates[$index]}" >&2
        done
        printf 'Select port [1]: ' >&2
        read -r choice
        choice="${choice:-1}"
        if ! [[ "$choice" =~ ^[0-9]+$ ]] || (( choice < 1 || choice > ${#candidates[@]} )); then
            printf 'Invalid selection.\n' >&2
            exit 2
        fi
        selected="${candidates[$((choice - 1))]}"
    else
        selected="${candidates[0]}"
        printf 'Multiple serial devices found; non-interactive mode selected %s.\n' "$selected" >&2
    fi

    printf 'Using serial port: %s\n' "$selected" >&2
    set -- --port "$selected" "$@"
fi

exec python3 "$SCRIPT_DIR/mili_voltron.py" "$@"
