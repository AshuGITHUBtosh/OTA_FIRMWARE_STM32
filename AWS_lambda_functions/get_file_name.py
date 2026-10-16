import json
import boto3

s3 = boto3.client("s3")

BUCKET = "modemtesting"

def lambda_handler(event, context):
    try:
        response = s3.list_objects_v2(Bucket=BUCKET)

        objects = response.get("Contents", [])

        firmware_files = [
            obj["Key"]
            for obj in objects
            if obj["Key"].lower().endswith(".bin")
        ]

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

        if len(firmware_files) > 1:
            return {
                "statusCode": 409,
                "headers": {
                    "Content-Type": "application/json"
                },
                "body": json.dumps({
                    "status": "error",
                    "message": "More than one firmware .bin file found"
                })
            }

        filename = firmware_files[0]

        print("Firmware filename:", filename)

        return {
            "statusCode": 200,
            "headers": {
                "Content-Type": "application/json"
            },
            "body": json.dumps({
                "status": "success",
                "filename": filename
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