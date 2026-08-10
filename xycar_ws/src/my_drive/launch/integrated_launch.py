"""Full rule drive with the 4-lamp signal and one shortcut selection."""

from my_drive.drive_launch import generate_drive_launch


def generate_launch_description():
    return generate_drive_launch(integrated=True)
