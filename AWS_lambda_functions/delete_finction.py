import json
import boto3

s3 = boto3.client("s3")

BUCKET = "modemtesting"


def lambda_handler(event, context):

    try:
        # List objects in the bucket
        response = s3.list_objects_v2(
            Bucket=BUCKET
        )

        objects = response.get("Contents", [])

        # Find .bin firmware files
        firmware_files = [
            obj["Key"]
            for obj in objects
            if obj["Key"].lower().endswith(".bin")
        ]

        # No firmware found
        if len(firmware_files) == 0:
            return {
                "statusCode": 404,
                "headers": {
                    "Content-Type": "application/json"
                },
                "body": json.dumps({
                    "status": "error",
                    "message": "No firmware .bin file found"
                })
            }

        # Safety check:
        # Never delete if there is more than one firmware file
        if len(firmware_files) > 1:
            return {
                "statusCode": 409,
                "headers": {
                    "Content-Type": "application/json"
                },
                "body": json.dumps({
                    "status": "error",
                    "message": "More than one firmware .bin file found",
                    "files": firmware_files
                })
            }

        # Exactly one firmware file
        firmware_key = firmware_files[0]

        print("Deleting firmware:", firmware_key)

        # Delete the firmware
        s3.delete_object(
            Bucket=BUCKET,
            Key=firmware_key
        )

        print("Firmware deleted successfully")

        return {
            "statusCode": 200,
            "headers": {
                "Content-Type": "application/json"
            },
            "body": json.dumps({
                "status": "success",
                "message": "Firmware deleted successfully",
                "deleted_file": firmware_key
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