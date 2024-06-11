#!/usr/bin/env bash

get_brightness() {
  brightnessctl -m | awk -F, '{print substr($4, 0, length($4)-1)}'
}

down() {
  brightnessctl s 5%-
  brightness=$(get_brightness)
  dunstify -a "BRIGHTNESS" "Decreasing to $brightness%" -h int:value:"$brightness" -i display-brightness-symbolic -r 2593 -u normal
}

up() {
  brightnessctl s 5%+
  brightness=$(get_brightness)
  dunstify -a "BRIGHTNESS" "Increasing to $brightness%" -h int:value:"$brightness" -i display-brightness-symbolic -r 2593 -u normal
}

case "$1" in
  up) up;;
  down) down;;
esac
