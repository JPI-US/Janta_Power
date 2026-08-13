Place tower certificates in this folder. These files should never be committed.

The following files are expected at build time, with `DEVICE_ID` configured in the `.env` file:
- `AmazonRootCA1.pem`
- `tower_{DEVICE_ID}-certificate.pem.crt`
- `tower_{DEVICE_ID}-private.pem.key`
