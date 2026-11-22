# pylint: disable=C0103,C0114,E0401,E1101,R0801,R0903,R0914

from ultralytics import YOLO
from ultralytics.data.split import autosplit

autosplit("labeling/images", weights=(0.8, 0.2, 0.0), annotated_only=True)
model = YOLO("yolo26n.pt")
model.train(data="training/balls.yaml", epochs=25, imgsz=512)  # type: ignore
