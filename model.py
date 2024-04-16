#!/usr/bin/env python3
from ros_robot.settings import DATA


class Model:
    def __init__(self) -> None:
        self.front_camera: list = [
            DATA['widthCamera'],
            DATA['heightCamera'],
            DATA['frontCamera']['index'],
            DATA['frontCamera']['comPort'],
            DATA['frontCamera']['modelPath'],
            DATA['frontCamera']['labelPath'],
            DATA['frontCamera']['threshold']
        ]
        
        self.omni_camera: list = [
            DATA['widthCamera'],
            DATA['heightCamera'],
            DATA['omniCamera']['index'],
            DATA['omniCamera']['comPort'],
            DATA['omniCamera']['xCenter'],
            DATA['omniCamera']['yCenter'],
            DATA['omniCamera']['lowerColor'],
            DATA['omniCamera']['upperColor'],
            DATA['omniCamera']['kernelValue'],
            DATA['omniCamera']['erodeValue'],
            DATA['omniCamera']['dilasiValue']
        ]

        self.depth: dict[int] = {
            'BOLA': DATA['depthData']['bola'],
            'ROBOT' : DATA['depthData']['robot'],
            'GAWANG': DATA['depthData']['gawang'],
            'PENGHALANG' : DATA['depthData']['penghalang']
        }

        self.focal_length: int = DATA['focalLength']
        self.real_distance: int = DATA['realDist']