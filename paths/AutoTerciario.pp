{
  "version": "2.0.0",
  "timestamp": "2026-11-06T21:35:15.480Z",
  "startPoint": {
    "x": 59.6258,
    "y": 136.8065,
    "locked": false,
    "headingDeg": 270
  },
  "lines": [
    {
      "kind": "compound",
      "id": "terciario-grupo-preparacao_flower",
      "name": "preparacao_flower",
      "color": "#76b7b2",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "segments": [
        {
          "kind": "atomic",
          "id": "terciario-prep_coleta_flower",
          "name": "prep_coleta_flower",
          "endPoint": {
            "x": 46.4006,
            "y": 120.9045
          },
          "controlPoints": [
            {
              "x": 67.8287,
              "y": 116.669
            }
          ],
          "heading": {
            "type": "linear",
            "startDeg": 270,
            "endDeg": 90
          },
          "color": "#b07aa1",
          "locked": false,
          "waitBeforeMs": 0,
          "waitAfterMs": 0,
          "waitBeforeName": "",
          "waitAfterName": ""
        }
      ]
    },
    {
      "kind": "atomic",
      "id": "terciario-coleta_flower",
      "name": "coleta_flower",
      "endPoint": {
        "x": 46.6,
        "y": 131
      },
      "controlPoints": [],
      "heading": {
        "type": "constant",
        "degrees": 90
      },
      "color": "#9c755f",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "kind": "atomic",
      "id": "terciario-pos_flower",
      "name": "pos_flower",
      "endPoint": {
        "x": 46.6,
        "y": 120
      },
      "controlPoints": [],
      "heading": {
        "type": "tangential",
        "reverse": true
      },
      "color": "#e15759",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "kind": "compound",
      "id": "terciario-grupo-retorno_lancamento_3",
      "name": "retorno_lancamento_3",
      "color": "#4e79a7",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "segments": [
        {
          "kind": "atomic",
          "id": "terciario-lancamento_3",
          "name": "lancamento_3",
          "endPoint": {
            "x": 59.8174,
            "y": 136.7026
          },
          "controlPoints": [
            {
              "x": 68.0113,
              "y": 117.3994
            }
          ],
          "heading": {
            "type": "tangential",
            "reverse": true
          },
          "color": "#f28e2b",
          "locked": false,
          "waitBeforeMs": 0,
          "waitAfterMs": 0,
          "waitBeforeName": "",
          "waitAfterName": ""
        }
      ]
    },
    {
      "kind": "atomic",
      "id": "terciario-estacionamento",
      "name": "estacionamento",
      "endPoint": {
        "x": 10,
        "y": 120
      },
      "controlPoints": [
        {
          "x": 65.729,
          "y": 114.1129
        }
      ],
      "heading": {
        "type": "linear",
        "startDeg": 270,
        "endDeg": 0
      },
      "color": "#59a14f",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [],
  "sequence": [
    {
      "kind": "wait",
      "id": "terciario-espera-aquecimento_mira_disparo_inicial",
      "name": "aquecimento_mira_disparo_inicial",
      "durationMs": 4000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "terciario-prep_coleta_flower"
    },
    {
      "kind": "path",
      "lineId": "terciario-coleta_flower"
    },
    {
      "kind": "wait",
      "id": "terciario-espera-coleta_flower_parada",
      "name": "coleta_flower_parada",
      "durationMs": 2000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "terciario-pos_flower"
    },
    {
      "kind": "wait",
      "id": "terciario-espera-recolher_flower",
      "name": "recolher_flower",
      "durationMs": 500,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "terciario-lancamento_3"
    },
    {
      "kind": "wait",
      "id": "terciario-espera-mira_disparo_retorno",
      "name": "mira_disparo_retorno",
      "durationMs": 2500,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "terciario-estacionamento"
    }
  ],
  "fieldPoints": [],
  "robot": {
    "forwardVelocity": 61.753623646754725,
    "strafeVelocity": 49.871306239265905,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rLength": 16,
    "rWidth": 16,
    "safetyMargin": 1,
    "maxAcceleration": 30,
    "maxDeceleration": 30
  },
  "activePaths": []
}
