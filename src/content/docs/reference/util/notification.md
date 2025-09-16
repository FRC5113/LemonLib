---
title: Notification
slug: reference/util/notification
---

# Notification

Represents an notification with various display properties.

## Methods

### __init__()

Initializes an ElasticNotification object.

Args:
    level (str): The severity level of the notification. Default is 'INFO'.
    title (str): The title of the notification. Default is an empty string.
    description (str): The description of the notification. Default is an empty string.
    display_time (int): Time in milliseconds for which the notification should be displayed. Default is 3000 ms.
    width (float): Width of the notification display area. Default is 350.
    height (float): Height of the notification display area. Default is -1 (automatic height).

