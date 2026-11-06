{
  "version": "2.0.0",
  "timestamp": "2026-11-06T21:21:15.774Z",
  "startPoint": {
    "x": 57.8,
    "y": 9,
    "locked": false,
    "headingDeg": 90
  },
  "lines": [
    {
      "kind": "compound",
      "id": "v3-grupo-ate_lancamento_2",
      "name": "ate_lancamento_2",
      "color": "#4e79a7",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "segments": [
        {
          "kind": "atomic",
          "id": "v3-chegadinha_frente_1",
          "name": "chegadinha_frente_1",
          "endPoint": {
            "x": 57.8,
            "y": 20
          },
          "controlPoints": [],
          "heading": {
            "type": "tangential",
            "reverse": false
          },
          "color": "#4e79a7",
          "locked": false,
          "waitBeforeMs": 0,
          "waitAfterMs": 0,
          "waitBeforeName": "",
          "waitAfterName": ""
        },
        {
          "kind": "atomic",
          "id": "v3-prep_coleta_jardim",
          "name": "prep_coleta_jardim",
          "endPoint": {
            "x": 31,
            "y": 13
          },
          "controlPoints": [],
          "heading": {
            "type": "linear",
            "startDeg": 90,
            "endDeg": 180
          },
          "color": "#f28e2b",
          "locked": false,
          "waitBeforeMs": 0,
          "waitAfterMs": 0,
          "waitBeforeName": "",
          "waitAfterName": ""
        },
        {
          "kind": "atomic",
          "id": "v3-coleta_jardim",
          "name": "coleta_jardim",
          "endPoint": {
            "x": 14,
            "y": 13
          },
          "controlPoints": [],
          "heading": {
            "type": "constant",
            "degrees": 180
          },
          "color": "#59a14f",
          "locked": false,
          "waitBeforeMs": 0,
          "waitAfterMs": 0,
          "waitBeforeName": "",
          "waitAfterName": ""
        },
        {
          "kind": "atomic",
          "id": "v3-lancamento_2",
          "name": "lancamento_2",
          "endPoint": {
            "x": 60,
            "y": 147
          },
          "controlPoints": [
            {
              "x": 38.1594,
              "y": 48.3839
            },
            {
              "x": 6.5729,
              "y": 108.8181
            },
            {
              "x": 57.3303,
              "y": 105.8968
            }
          ],
          "heading": {
            "type": "linear",
            "startDeg": 180,
            "endDeg": 270
          },
          "color": "#edc948",
          "locked": false,
          "waitBeforeMs": 0,
          "waitAfterMs": 0,
          "waitBeforeName": "",
          "waitAfterName": ""
        }
      ]
    },
    {
      "kind": "compound",
      "id": "v3-grupo-preparacao_flower",
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
          "id": "v3-prep_coleta_flower",
          "name": "prep_coleta_flower",
          "endPoint": {
            "x": 50.6,
            "y": 122
          },
          "controlPoints": [
            {
              "x": 67.82870967741937,
              "y": 116.66903225806453
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
      "id": "v3-coleta_flower",
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
      "id": "v3-pos_flower",
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
      "id": "v3-grupo-retorno_lancamento_3",
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
          "id": "v3-pre_lancamento_3",
          "name": "pre_lancamento_3",
          "endPoint": {
            "x": 60,
            "y": 122
          },
          "controlPoints": [],
          "heading": {
            "type": "linear",
            "startDeg": 90,
            "endDeg": 270
          },
          "color": "#4e79a7",
          "locked": false,
          "waitBeforeMs": 0,
          "waitAfterMs": 0,
          "waitBeforeName": "",
          "waitAfterName": ""
        },
        {
          "kind": "atomic",
          "id": "v3-lancamento_3",
          "name": "lancamento_3",
          "endPoint": {
            "x": 60,
            "y": 147
          },
          "controlPoints": [],
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
      "id": "v3-estacionamento",
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
      "id": "v3-espera-aquecimento_mira_disparo_inicial",
      "name": "aquecimento_mira_disparo_inicial",
      "durationMs": 4000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "v3-chegadinha_frente_1"
    },
    {
      "kind": "path",
      "lineId": "v3-prep_coleta_jardim"
    },
    {
      "kind": "path",
      "lineId": "v3-coleta_jardim"
    },
    {
      "kind": "path",
      "lineId": "v3-lancamento_2"
    },
    {
      "kind": "wait",
      "id": "v3-espera-mira_disparo_2",
      "name": "mira_disparo_2",
      "durationMs": 2500,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "v3-prep_coleta_flower"
    },
    {
      "kind": "path",
      "lineId": "v3-coleta_flower"
    },
    {
      "kind": "wait",
      "id": "v3-espera-coleta_flower_parada",
      "name": "coleta_flower_parada",
      "durationMs": 2000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "v3-pos_flower"
    },
    {
      "kind": "wait",
      "id": "v3-espera-recolher_flower",
      "name": "recolher_flower",
      "durationMs": 500,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "v3-pre_lancamento_3"
    },
    {
      "kind": "path",
      "lineId": "v3-lancamento_3"
    },
    {
      "kind": "wait",
      "id": "v3-espera-mira_disparo_3",
      "name": "mira_disparo_3",
      "durationMs": 2500,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "v3-estacionamento"
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
