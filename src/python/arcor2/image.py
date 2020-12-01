import io
import json
from typing import Optional

import PIL.Image
import numpy as np
import cv2
from PIL.Image import Image

ENCODING = "latin-1"


def image_to_cv2(pil_image: Image) -> np.array:

    return cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)


def image_to_bytes_io(value: Image, target_format: str = "jpeg", target_mode: Optional[str] = None) -> io.BytesIO:

    output = io.BytesIO()

    if target_mode and value.mode != target_mode:
        rgb_im = value.convert(target_mode)
        rgb_im.save(output, target_format)
    else:
        value.save(output, target_format)
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
