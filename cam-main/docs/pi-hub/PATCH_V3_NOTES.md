# Patch v3 notes

Adds explicit hotspot internet sharing support for OTA while preserving pre-imaged Pi identity/system settings.

Relevant settings in `Rpi/conf/pi_hub.conf`:

```bash
PI_HUB_HOTSPOT_ENABLE_NAT=1
PI_HUB_HOTSPOT_NET_IFACE="auto"
PI_HUB_HOTSPOT_NAT_WAIT_SEC=25
PI_HUB_HOTSPOT_DNS_MODE="system"
```

The updated `setup_hotspot.sh` waits for an upstream default route, enables IPv4 forwarding, adds NAT/forwarding rules, and records the upstream interface so stop/restart cleanup is reliable.
