import boto3
import base64

s3 = boto3.client("s3")

BUCKET = "modemtesting"


def lambda_handler(event, context):

    try:

        # Find files in the bucket
        response = s3.list_objects_v2(
            Bucket=BUCKET
        )

        objects = response.get("Contents", [])

        # Find .bin files
        firmware_files = [
            obj for obj in objects
            if obj["Key"].lower().endswith(".bin")
        ]

        # No firmware
        if len(firmware_files) == 0:
            return {
                "statusCode": 404,
                "headers": {
                    "Content-Type": "text/plain"
                },
                "body": "No firmware .bin file found"
            }

        # More than one firmware
        if len(firmware_files) > 1:
            return {
                "statusCode": 409,
                "headers": {
                    "Content-Type": "text/plain"
                },
                "body": "More than one firmware .bin file found"
            }

        # Select the only firmware file
        KEY = firmware_files[0]["Key"]

        print("Firmware selected:", KEY)

        # Download firmware
        response = s3.get_object(
            Bucket=BUCKET,
            Key=KEY
        )

        firmware = response["Body"].read()

        print("Firmware size:", len(firmware), "bytes")

        # Return firmware in the SAME format as before
        return {
            "statusCode": 200,
            "headers": {
                "Content-Type": "application/octet-stream"
            },
            "body": base64.b64encode(firmware).decode("utf-8"),
            "isBase64Encoded": True
        }

    except Exception as e:

        print("ERROR:", str(e))

        return {
            "statusCode": 500,
            "headers": {
                "Content-Type": "text/plain"
            },
            "body": "Firmware download failed: " + str(e)
        }