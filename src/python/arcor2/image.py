import base64
import io
import json

import PIL.Image  # type: ignore
from PIL.Image import Image  # type: ignore


def image_to_base64(value: Image) -> str:

    with io.BytesIO() as output:
        value.save(output, "jpeg")
        b64_bytes = base64.b64encode(output.getvalue())
        return b64_bytes.decode()


def image_to_json(value: Image) -> str:
    return json.dumps(image_to_base64(value))


def image_from_base64(value: str) -> Image:

    b64_bytes = value.encode()
    image_data = base64.b64decode(b64_bytes)
    return PIL.Image.open(io.BytesIO(image_data))


def image_from_json(value: str) -> Image:
    return image_from_base64(json.loads(value))
