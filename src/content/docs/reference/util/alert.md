---
title: Alert
slug: reference/util/alert
---

# Alert

Represents an individual alert with text, type, and optional timeout.

Alerts can be activated, deactivated, or updated with new text.

## Methods

### __init__()

Initialize an alert instance.

Args:
    text (str): The message text for the alert.
    type (AlertType): The severity level of the alert.
    timeout (float): Duration in seconds after which the alert auto-deactivates.
    elasticnoti (bool): Whether to send the alert to the Elastic dashboard. defaults to True.

### set()

Activate or deactivate the alert.

Args:
    active (bool): True to activate, False to deactivate.

### enable()

Enable the alert.

### disable()

Disable the alert.

### set_text()

Update the alert's text and log the change if it is active.

Args:
    text (str): New text for the alert.

