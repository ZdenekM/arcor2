import io
import json

import PIL.Image
from PIL.Image import Image

TARGET_MODE = "RGB"
TARGET_FORMAT = "JPEG"
ENCODING = "latin-1"


def image_to_bytes_io(value: Image) -> io.BytesIO:

    output = io.BytesIO()

    if value.mode != TARGET_MODE:
        rgb_im = value.convert(TARGET_MODE)
        rgb_im.save(output, TARGET_FORMAT)
    else:
        value.save(output, TARGET_FORMAT)
    output.seek(0)
    return output


def image_to_str(value: Image) -> str:
    return image_to_bytes_io(value).getvalue().decode(ENCODING)


def image_from_str(value: str) -> Image:
    return image_to_bytes_io(io.BytesIO(value.encode(ENCODING)))


def image_to_json(value: Image) -> str:
    return json.dumps(image_to_str(value))


def image_from_bytes_io(value: io.BytesIO) -> Image:
    value.seek(0)
    return PIL.Image.open(value)


def image_from_json(value: str) -> Image:
    return image_from_str(json.loads(value))
