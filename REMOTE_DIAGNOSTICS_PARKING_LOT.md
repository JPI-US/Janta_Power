# Remote Diagnostics Parking Lot

These are parked ideas from the first MQTT diagnostics acknowledgement test.

## What Worked

- AWS can publish to the tower-specific command topic.
- The tower can receive the command.
- The tower can publish an acknowledgement back to AWS.
- Test topic shape:
  - command: `tower/1/cmd/diagnostics`
  - acknowledgement: `tower/1/cmd/diagnostics/ack`

Example command:

```json
{
  "cmd": "ping",
  "request_id": "test-001",
  "message": "Hello from AWS IoT console"
}
```

Example acknowledgement:

```json
{
  "current_time": "30/04/2026 15:01:42",
  "request_id": "test-001",
  "cmd": "ping",
  "status": "received",
  "message": "Tower received diagnostics command"
}
```

## Duplicate Acknowledgement Fixes

The tower appeared to acknowledge the same command more than once. Likely causes:

- The AWS command may have been published as a retained message.
- The same `request_id` was reused.
- The firmware currently has no duplicate-request guard.
- Diagnostics is currently polled from the main tracking loop, so queued duplicates may show up later.

Possible fixes:

- Confirm AWS IoT Console retain is off for command publishes.
- Require a unique `request_id` for each command.
- Track recently processed `request_id` values in RAM.
- Ignore duplicate `request_id` commands during the same boot.
- Later, poll diagnostics more frequently than the 300-second tracking loop.

## Temporary Admin Mode Idea

Start with read-only diagnostics, then add temporary admin mode for risky commands.

Suggested staged flow:

1. Stabilize the command inbox.
2. Add read-only commands:
   - `ping`
   - `get_status`
   - `get_time`
   - `get_encoder`
   - `get_config_summary`
3. Add temporary admin unlock:

```json
{
  "cmd": "admin_unlock",
  "request_id": "admin-001",
  "ttl_seconds": 300
}
```

4. Keep temporary admin state in RAM only.
5. Auto-expire admin mode after the TTL.
6. Gate risky commands behind temporary admin mode:
   - `jog_motor`
   - `rehome`
   - `run_motor_test`
   - `set_mode`
   - `set_config`
7. Publish an ack/log for every admin action with:
   - `request_id`
   - `cmd`
   - accepted/rejected status
   - reason
   - admin remaining seconds

## Guiding Principle

Remote diagnostics should start as observability, not remote control.

The first stable milestone is: AWS asks the tower a question, and the tower gives one structured answer exactly once.
