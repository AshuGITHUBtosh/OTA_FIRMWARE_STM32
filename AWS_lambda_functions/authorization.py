import json

def lambda_handler(event, context):
    # TODO implement
    return {
        'statusCode': 200,
        'body': json.dumps('Hello from Lambda!')
    }
import os

SECRET = os.environ["OTA_SECRET"]

def lambda_handler(event, context):

    headers = event.get("headers", {})

    # HTTP header names are normally case-insensitive,
    # but API Gateway generally provides them in lowercase.
    token = headers.get("authorization")

    if token == SECRET:
        return {
            "isAuthorized": True
        }

    return {
        "isAuthorized": False
    }