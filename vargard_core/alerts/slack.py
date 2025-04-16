"""
Slack alert sender using incoming webhook.
"""
import requests

def send_slack(webhook_url: str, message: str):
    """
    Send a text message to a Slack channel via incoming webhook.
    """
    payload = {"text": message}
    try:
        response = requests.post(webhook_url, json=payload, timeout=2.0)
        response.raise_for_status()
    except Exception as e:
        # Let caller handle logging
        raise RuntimeError(f"Slack webhook failed: {e}")