#!/bin/bash
#
# Sends Telgraph message to a given Chat Id with information when an event happens.
#
# Make Exciutable chmod +x notifycmd.sh
#
# This script can be tested without sending an email by running as follows:
#     UPSNAME="eatonups1@localhost" ./notifycmd
#
set -e

# Replace with your Telegram Bot API token
BOT_TOKEN="<BOTTOKEN>"
# Replace with your Telegram chat ID (your own or a group/channel)
CHAT_ID="<CHATID>"

# UPS Info
BATT=$(upsc "${UPSNAME}" battery.charge)
BATT_LOW=$(upsc "${UPSNAME}" battery.charge.low)
BATT_RUNTIME=$(upsc "${UPSNAME}" battery.runtime)
MODEL=$(upsc "${UPSNAME}" ups.model)
SERIAL=$(upsc "${UPSNAME}" ups.serial)
LOAD=$(upsc "${UPSNAME}" ups.load)
STATUS=$(upsc "${UPSNAME}" ups.status)

# Prepare email subject and body
BODY=$(cat <<EOF
UPS: ${UPSNAME}
Model: ${MODEL}
Serial: ${SERIAL}

Status: ${STATUS}
Load: ${LOAD}%
Battery Charge: ${BATT}%
Battery Charge Low Threshold: ${BATT_LOW}%
Battery Runtime: $(date -d@"${BATT_RUNTIME}" -u "+%M mins %S seconds") (${BATT_RUNTIME})
EOF
)

echo "Prepairing to notify with the following message:"
echo "${BODY}"
echo -e "\n"

# Telegram API endpoint for sending messages
TELEGRAM_API="https://api.telegram.org/bot$BOT_TOKEN/sendMessage"

# Send the message using curl
curl -s -X POST $TELEGRAM_API -d chat_id=$CHAT_ID -d text="$BODY"

echo "Done!"