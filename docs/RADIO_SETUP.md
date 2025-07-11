# FRC Radio Configuration Guide

## Event vs Home Programming
- **At FRC Events:** Use the Vivid-Hosting Radio Kiosk
- **At Home:** Follow manual configuration steps below

## VH-109 Radio Setup Instructions

### 1. Computer Network Configuration Options

#### Option A: DHCP (Recommended)
- Radio accessible at:
  - Client/STA mode: `10.XX.YY.1`
  - AP mode: `10.XX.YY.4`
  - Factory default: `10.0.1.1`

#### Option B: Manual Static IP
- Configure network adapter manually if DHCP fails

### 2. Radio Connection Methods

#### Wired Connection
- Connect via Ethernet cable
- Access URL: `http://192.168.69.1/`

#### Wireless Connection
- Factory Default: `http://10.0.1.1/`
- Team Number Set: `http://10.XX.YY.1/`
- mDNS (Firmware >1.1.0): `http://radio.local/`

### 3. Radio Configuration Parameters

| Setting | Description | Notes |
|---------|-------------|-------|
| Mode Selection | Robot Radio (Client/STA) or Access Point | Choose based on usage |
| Team Number | Your FRC team number (5607) | Required |
| SSID Suffix | Optional identifier (e.g., "5607-comp") | Appended to SSID |
| 6GHz WPA Key | Security key for 6GHz connection | Required |
| 2.4GHz WPA Key | Security key for 2.4GHz AP mode | Required if using AP |

### 4. DIP Switch Settings

**Important Note:** DIP switch #3 controls 2.4GHz AP behavior:
- ON: Enables 2.4GHz fallback mode
- Creates SSID: `FRC-XXYY[-SUFFIX]`
- Deactivates when 6GHz connection established

### 5. Best Practices
1. Use dedicated AP mode radio for practice
2. Avoid 2.4GHz in congested environments
3. Regular firmware updates recommended
4. Document radio configuration settings
5. Test connection before competitions

### 6. Troubleshooting
- Verify power connection
- Check network adapter settings
- Confirm firmware version
- Test both wired and wireless access
- Factory reset if necessary
https://frc-radio.vivid-hosting.net/overview/programming-your-radio-at-home