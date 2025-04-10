#!/usr/bin/env bash

down() {
  pamixer -d 2
  volume=$(pamixer --get-volume) 
  dunstify -a "VOLUME" "Decreasing to $volume%" -h int:value:"$volume" -i audio-volume-low-symbolic -r 2593 -u normal
}

up() {
  pamixer -i 2
  volume=$(pamixer --get-volume)
  dunstify -a "VOLUME" "Increasing to $volume%" -h int:value:"$volume" -i audio-volume-high-symbolic -r 2593 -u normal
}

mute() {
  muted="$(pamixer --get-mute)"
  if $muted; then
    pamixer -u
    dunstify -a "VOLUME" "UNMUTED" -i audio-volume-high-symbolic -r 2593 -u normal
  else 
    pamixer -m
    dunstify -a "VOLUME" "MUTED" -i audio-volume-muted-symbolic -r 2593 -u normal
  fi
}

case "$1" in
  up) up;;
  down) down;;
  mute) mute;;
esac
