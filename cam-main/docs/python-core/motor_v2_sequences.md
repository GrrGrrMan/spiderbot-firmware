# Motor V2 sequence examples

Motor V2 stores programs in ESP RAM only. A reboot clears them. Send commands to:

```text
alphaesp32s3/spiderbot-s3/cmd/motor
```

The same command body also works through the normal command topic with the
`motor:` prefix:

```text
alphaesp32s3/spiderbot-s3/cmd
```

## Two-servo sweep loop

This sends one setup command. The ESP runs the loop locally after receiving it.

```text
load sweep loops=0; pose 1000 0=90 1=90; pose 1000 0=180 1=180; run
```

Equivalent command-topic form:

```text
motor:load sweep loops=0; pose 1000 0=90 1=90; pose 1000 0=180 1=180; run
```

`loops=0` means loop forever. Use `motor:stop` to stop and hold the current
servo positions, or `motor:stop free` to release all PWM outputs.

Multi-channel `pose` steps are intentionally staggered by
`CFG_MOTOR_V2_POSE_CHANNEL_STAGGER_MS` between channel writes. The default is
8 ms, so a two-channel `pose 1000 ...` finishes in about 1008 ms instead of
starting both servo PWM changes in the same firmware instant.

Motor state is quiet by default. Ask for one snapshot with:

```text
motor:status
```

Enable a short leased state stream for a controller session with:

```text
motor:stream status on
```

Disable it with:

```text
motor:stream status off
```

The legacy command prefix also works:

```text
servo:stream:status:on
```

## Build and run in pieces

```text
motor:load nod loops=3
motor:add nod pose 500 0=70 1=110
motor:add nod pose 500 0=110 1=70
motor:run nod
```

## Direct movement

```text
motor:move 0 120 750
motor:pose 1000 0=90 1=120 2=60
motor:center all 500
motor:free all
```

## Inspection

```text
motor:list
motor:show sweep
motor:status
motor:params
```

## Rejections

The firmware rejects out-of-scope values instead of clamping them silently.

```text
motor:move 0 270 1000
```

That fails because the target angle must stay within `0..180`.
