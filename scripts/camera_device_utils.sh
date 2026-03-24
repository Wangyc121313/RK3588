#!/usr/bin/env bash

is_capture_device() {
	local device="$1"
	[[ -e "$device" ]] || return 1

	if command -v udevadm >/dev/null 2>&1; then
		local props
		props="$(udevadm info -q property -n "$device" 2>/dev/null || true)"
		if [[ -n "$props" ]]; then
			grep -q 'ID_V4L_CAPABILITIES=.*:capture:' <<< "$props"
			return
		fi
	fi

	return 0
}

resolve_camera_device() {
	local requested="${1:-auto}"
	local candidate

	if [[ -n "$requested" && "$requested" != "auto" ]]; then
		if [[ -e "$requested" ]]; then
			echo "$requested"
			return 0
		fi
		echo "camera device not found: $requested" >&2
		return 1
	fi

	for candidate in /dev/v4l/by-id/*-video-index0; do
		[[ -e "$candidate" ]] || continue
		if is_capture_device "$candidate"; then
			echo "$candidate"
			return 0
		fi
	done

	for candidate in /dev/v4l/by-path/*-video-index0; do
		[[ -e "$candidate" ]] || continue
		if is_capture_device "$candidate"; then
			echo "$candidate"
			return 0
		fi
	done

	for candidate in /dev/video[0-9]*; do
		[[ -e "$candidate" ]] || continue
		if is_capture_device "$candidate"; then
			echo "$candidate"
			return 0
		fi
	done

	echo "no capture-capable camera device found" >&2
	return 1
}