import json
import boto3
import base64

s3 = boto3.client("s3")

BUCKET_NAME = "modemtesting"


def lambda_handler(event, context):

    try:

        body = event.get("body", "")

        if body is None:
            body = ""

        if event.get("isBase64Encoded", False):
            body = base64.b64decode(body)

        if isinstance(body, bytes):
            body = body.decode("utf-8")

        firmware_filename = body.strip()

        print("Received firmware filename:",
              firmware_filename)

        # Basic validation
        if not firmware_filename:
            return {
                "statusCode": 400,
                "body": json.dumps({
                    "status": "error",
                    "message": "Firmware filename missing"
                })
            }

        if not firmware_filename.lower().endswith(".bin"):
            return {
                "statusCode": 400,
                "body": json.dumps({
                    "status": "error",
                    "message": "Invalid firmware filename"
                })
            }

        if "/" in firmware_filename or \
           "\\" in firmware_filename or \
           ".." in firmware_filename:

            return {
                "statusCode": 400,
                "body": json.dumps({
                    "status": "error",
                    "message": "Invalid firmware filename"
                })
            }

        success_filename = (
            "success_" + firmware_filename + ".txt"
        )

        s3.put_object(
            Bucket=BUCKET_NAME,
            Key=success_filename,
            Body=firmware_filename.encode("utf-8"),
            ContentType="text/plain"
        )

        print(
            "Created success file:",
            success_filename
        )

        return {
            "statusCode": 200,
            "headers": {
                "Content-Type": "application/json"
            },
            "body": json.dumps({
                "status": "success",
                "message": "Firmware update success received",
                "s3_object": success_filename
            })
        }

    except Exception as e:

        print("ERROR:", str(e))

        return {
            "statusCode": 500,
            "headers": {
                "Content-Type": "application/json"
            },
            "body": json.dumps({
                "status": "error",
                "message": str(e)
            })
        }