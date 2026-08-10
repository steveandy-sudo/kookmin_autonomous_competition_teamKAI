"""Lane, cone, and vehicle-avoidance driving without signal missions."""

from my_drive.drive_launch import generate_drive_launch


def generate_launch_description():
    return generate_drive_launch(integrated=False)
