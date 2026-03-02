{
  "version": "1.7.4",
  "header": {
    "info": "Created with Pedro Pathing Plus Visualizer",
    "copyright": "Copyright 2026 Matthew Allen. Licensed under the Modified Apache License, Version 2.0.",
    "link": "https://github.com/Mallen220/PedroPathingPlusVisualizer"
  },
  "startPoint": {
    "x": 16,
    "y": 113,
    "heading": "linear",
    "startDeg": 180,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-r06uysgxct",
      "name": "PreLoadEFlileiraMeio",
      "endPoint": {
        "x": 24.055443329085993,
        "y": 59.49504304316747,
        "heading": "constant",
        "startDeg": 180,
        "endDeg": 180,
        "degrees": 180
      },
      "controlPoints": [
        {
          "x": 89.54455045529414,
          "y": 66.87919942816296
        }
      ],
      "color": "#857A69",
      "eventMarkers": [],
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mm72vx9c-ppeg7v",
      "name": "ShootPose",
      "endPoint": {
        "x": 51.90695217080522,
        "y": 79.75644093607237,
        "heading": "linear",
        "reverse": true,
        "startDeg": 180,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#7C85AB",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mm732us9-r4w9b9",
      "name": "Gate",
      "endPoint": {
        "x": 16,
        "y": 64.64157628119465,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [
        {
          "x": 43.83565901109489,
          "y": 63.81782018211766
        }
      ],
      "color": "#A8CD5D",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mm739nib-tmhg7x",
      "name": "GiradinhaGate",
      "endPoint": {
        "x": 12.419802964850664,
        "y": 59.49504304316747,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 145
      },
      "controlPoints": [],
      "color": "#77C9DA",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mm73iwq8-7dxvsw",
      "name": "ShootGate",
      "endPoint": {
        "x": 51.90695217080522,
        "y": 79.75644093607237,
        "heading": "linear",
        "reverse": true,
        "startDeg": 145,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#996DBB",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mm745ywx-7c4mbw",
      "name": "PrimeiraFileira",
      "endPoint": {
        "x": 24.055443329085993,
        "y": 83.9049320943531,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [],
      "color": "#D7956B",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 144,
          "y": 69.5
        },
        {
          "x": 144,
          "y": 144
        },
        {
          "x": 119,
          "y": 144
        },
        {
          "x": 137.5,
          "y": 119
        },
        {
          "x": 137.5,
          "y": 69.5
        }
      ],
      "color": "#dc2626",
      "fillColor": "#fca5a5",
      "type": "obstacle"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6.5,
          "y": 119
        },
        {
          "x": 25,
          "y": 144
        },
        {
          "x": 0,
          "y": 144
        },
        {
          "x": 0,
          "y": 69.5
        },
        {
          "x": 6.5,
          "y": 69.5
        }
      ],
      "color": "#0b08d9",
      "fillColor": "#fca5a5",
      "type": "obstacle"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-r06uysgxct"
    },
    {
      "kind": "path",
      "lineId": "mm72vx9c-ppeg7v"
    },
    {
      "kind": "path",
      "lineId": "mm732us9-r4w9b9"
    },
    {
      "kind": "path",
      "lineId": "mm739nib-tmhg7x"
    },
    {
      "kind": "wait",
      "id": "mm74dl5s-yihn8p",
      "name": "",
      "durationMs": 6000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "mm73iwq8-7dxvsw"
    },
    {
      "kind": "path",
      "lineId": "mm745ywx-7c4mbw"
    }
  ]
}