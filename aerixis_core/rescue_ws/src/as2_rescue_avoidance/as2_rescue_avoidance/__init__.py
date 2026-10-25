"""Drone-to-drone avoidance for the rescue fleet.

orca       ORCA velocity filter: the safety shield that runs onboard (pure Python, no numpy).
links      mesh link models: base station (IP, main) and LoRa (long-range fallback).
sim        multi-drone training/evaluation simulator (laptop only).
scenarios  encounter generators: head-on on the same path, crossing, overtaking, blocker, random.
evaluate   runs policies over many scenarios and writes a report.
"""
