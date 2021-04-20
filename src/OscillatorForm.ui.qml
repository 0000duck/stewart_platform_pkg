import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.0
import QtQuick.Controls.Styles 1.4

PageBackground {
    width: 800
    height: 480
    property alias tfreqSlider: tfreqSlider
    property alias rfreqSlider: rfreqSlider
    property alias yawSlider: yawSlider
    property alias pitchSlider: pitchSlider
    property alias rollSlider: rollSlider
    property alias heaveSlider: heaveSlider
    property alias swaySlider: swaySlider
    property alias surgeSlider: surgeSlider
    clip: true
    visible: true
    transformOrigin: Item.Center
    title: "Oscillator"
    //Trans freq
    Slider {
        id: tfreqSlider
        x: 475
        y: 174
        width: 259
        height: 38
        stepSize: 0.1
        rotation: 270
        value: 0

        background: Rectangle {
            x: tfreqSlider.leftPadding
            y: tfreqSlider.topPadding + tfreqSlider.availableHeight / 2 - height / 2
            implicitWidth: 200
            implicitHeight: 4
            width: tfreqSlider.availableWidth
            height: implicitHeight
            radius: 2
            color: "#bdbebf"

            Rectangle {
                width: tfreqSlider.visualPosition * parent.width
                height: parent.height
                color: "#db6221"
                radius: 2
            }
        }

        handle: Rectangle {
            x: tfreqSlider.leftPadding + tfreqSlider.visualPosition * (tfreqSlider.availableWidth - width)
            y: tfreqSlider.topPadding + tfreqSlider.availableHeight / 2 - height / 2
            implicitWidth: 26
            implicitHeight: 26
            radius: 13
            color: tfreqSlider.pressed ? "#f0f0f0" : "#f6f6f6"
            border.color: "#bdbebf"
        }
    }

    ColumnLayout {
    }

    Button {
        id: startButton
        x: 69
        y: 390
        width: 131
        height: 55
        objectName: "start"
        text: qsTr("START")
        font.bold: true
        font.pointSize: 14
        font.family: "Arial"
    }

    Button {
        id: stopButton
        x: 252
        y: 390
        width: 131
        height: 55
        objectName: "stop"
        text: qsTr("STOP")
        hoverEnabled: true
        checked: false
        checkable: false
        autoExclusive: false
        font.pointSize: 14
        font.family: "Arial"
        font.bold: true
    }

    Label {
        id: label6
        x: 567
        y: 349
        color: "#f2f2f3"
        text: qsTr("TRANS FREQ")
    }

    Label {
        id: label7
        x: 666
        y: 349
        color: "#f4f4f5"
        text: qsTr("ROT FREQ")
    }
//Rot freq
    Slider {
        id: rfreqSlider
        x: 567
        y: 175
        width: 259
        height: 38
        stepSize: 0.1
        rotation: 270
        handle: Rectangle {
            x: rfreqSlider.leftPadding + rfreqSlider.visualPosition * (rfreqSlider.availableWidth - width)
            y: rfreqSlider.topPadding + rfreqSlider.availableHeight / 2 - height / 2
            color: rfreqSlider.pressed ? "#f0f0f0" : "#f6f6f6"
            radius: 13
            border.color: "#bdbebf"
            implicitHeight: 26
            implicitWidth: 26
        }
        value: 0
        background: Rectangle {
            x: rfreqSlider.leftPadding
            y: rfreqSlider.topPadding + rfreqSlider.availableHeight / 2 - height / 2
            width: rfreqSlider.availableWidth
            height: implicitHeight
            color: "#bdbebf"
            radius: 2
            implicitHeight: 4
            Rectangle {
                width: rfreqSlider.visualPosition * parent.width
                height: parent.height
                color: "#db6221"
                radius: 2
            }
            implicitWidth: 200
        }
    }
 //Yaw
    Slider {
        id: yawSlider
        x: 315
        y: 176
        width: 259
        height: 38
        stepSize: 0.1
        rotation: 270
        handle: Rectangle {
            x: yawSlider.leftPadding + yawSlider.visualPosition * (yawSlider.availableWidth - width)
            y: yawSlider.topPadding + yawSlider.availableHeight / 2 - height / 2
            color: yawSlider.pressed ? "#f0f0f0" : "#f6f6f6"
            radius: 13
            border.color: "#bdbebf"
            implicitHeight: 26
            implicitWidth: 26
        }
        value: 0
        background: Rectangle {
            x: yawSlider.leftPadding
            y: yawSlider.topPadding + yawSlider.availableHeight / 2 - height / 2
            width: yawSlider.availableWidth
            height: implicitHeight
            color: "#bdbebf"
            radius: 2
            implicitHeight: 4
            Rectangle {
                width: yawSlider.visualPosition * parent.width
                height: parent.height
                color: "#db6221"
                radius: 2
            }
            implicitWidth: 200
        }
    }
//Pitch
    Slider {
        id: pitchSlider
        x: 240
        y: 177
        width: 259
        height: 38
        stepSize: 0.1
        layer.smooth: false
        antialiasing: false
        rotation: 270
        handle: Rectangle {
            x: pitchSlider.leftPadding + pitchSlider.visualPosition * (pitchSlider.availableWidth - width)
            y: pitchSlider.topPadding + pitchSlider.availableHeight / 2 - height / 2
            color: pitchSlider.pressed ? "#f0f0f0" : "#f6f6f6"
            radius: 13
            border.color: "#bdbebf"
            implicitHeight: 26
            implicitWidth: 26
        }
        value: 0
        background: Rectangle {
            x: pitchSlider.leftPadding
            y: pitchSlider.topPadding + pitchSlider.availableHeight / 2 - height / 2
            width: pitchSlider.availableWidth
            height: implicitHeight
            color: "#bdbebf"
            radius: 2
            implicitHeight: 4
            Rectangle {
                width: pitchSlider.visualPosition * parent.width
                height: parent.height
                color: "#db6221"
                radius: 2
            }
            implicitWidth: 200
        }
    }
//Roll
    Slider {
        id: rollSlider
        x: 165
        y: 178
        width: 259
        height: 38
        stepSize: 0.1
        rotation: 270
        handle: Rectangle {
            x: rollSlider.leftPadding + rollSlider.visualPosition * (rollSlider.availableWidth - width)
            y: rollSlider.topPadding + rollSlider.availableHeight / 2 - height / 2
            color: rollSlider.pressed ? "#f0f0f0" : "#f6f6f6"
            radius: 13
            border.color: "#bdbebf"
            implicitHeight: 26
            implicitWidth: 26
        }
        value: 0
        background: Rectangle {
            x: rollSlider.leftPadding
            y: rollSlider.topPadding + rollSlider.availableHeight / 2 - height / 2
            width: rollSlider.availableWidth
            height: implicitHeight
            color: "#bdbebf"
            radius: 2
            implicitHeight: 4
            Rectangle {
                width: rollSlider.visualPosition * parent.width
                height: parent.height
                color: "#db6221"
                radius: 2
            }
            implicitWidth: 200
        }
    }
//Heave
    Slider {
        id: heaveSlider
        x: 90
        y: 179
        width: 259
        height: 38
        stepSize: 0.1
        rotation: 270
        handle: Rectangle {
            x: heaveSlider.leftPadding + heaveSlider.visualPosition * (heaveSlider.availableWidth - width)
            y: heaveSlider.topPadding + heaveSlider.availableHeight / 2 - height / 2
            color: heaveSlider.pressed ? "#f0f0f0" : "#f6f6f6"
            radius: 13
            border.color: "#bdbebf"
            implicitHeight: 26
            implicitWidth: 26
        }
        value: 0
        background: Rectangle {
            x: heaveSlider.leftPadding
            y: heaveSlider.topPadding + heaveSlider.availableHeight / 2 - height / 2
            width: heaveSlider.availableWidth
            height: implicitHeight
            color: "#bdbebf"
            radius: 2
            implicitHeight: 4
            Rectangle {
                width: heaveSlider.visualPosition * parent.width
                height: parent.height
                color: "#db6221"
                radius: 2
            }
            implicitWidth: 200
        }
    }
//Sway
    Slider {
        id: swaySlider
        x: 15
        y: 180
        width: 259
        height: 38
        stepSize: 0.1
        rotation: 270
        handle: Rectangle {
            x: swaySlider.leftPadding + swaySlider.visualPosition * (swaySlider.availableWidth - width)
            y: swaySlider.topPadding + swaySlider.availableHeight / 2 - height / 2
            color: swaySlider.pressed ? "#f0f0f0" : "#f6f6f6"
            radius: 13
            border.color: "#bdbebf"
            implicitHeight: 26
            implicitWidth: 26
        }
        value: 0
        background: Rectangle {
            x: swaySlider.leftPadding
            y: swaySlider.topPadding + swaySlider.availableHeight / 2 - height / 2
            width: swaySlider.availableWidth
            height: implicitHeight
            color: "#bdbebf"
            radius: 2
            implicitHeight: 4
            Rectangle {
                width: swaySlider.visualPosition * parent.width
                height: parent.height
                color: "#db6221"
                radius: 2
            }
            implicitWidth: 200
        }
    }
//Surge
    Slider {
        id: surgeSlider
        x: -60
        y: 179
        width: 259
        height: 38
        snapMode: Slider.SnapOnRelease
        wheelEnabled: false
        stepSize: 0.1
        rotation: 270
        handle: Rectangle {
            x: surgeSlider.leftPadding + surgeSlider.visualPosition * (surgeSlider.availableWidth - width)
            y: surgeSlider.topPadding + surgeSlider.availableHeight / 2 - height / 2
            color: surgeSlider.pressed ? "#f0f0f0" : "#f6f6f6"
            radius: 13
            border.color: "#bdbebf"
            implicitHeight: 26
            implicitWidth: 26
        }
        value: 0
        background: Rectangle {
            x: surgeSlider.leftPadding
            y: surgeSlider.topPadding + surgeSlider.availableHeight / 2 - height / 2
            width: surgeSlider.availableWidth
            height: implicitHeight
            color: "#bdbebf"
            radius: 2
            implicitHeight: 4
            Rectangle {
                width: surgeSlider.visualPosition * parent.width
                height: parent.height
                color: "#db6221"
                radius: 2
            }
            implicitWidth: 200
        }
    }

    Label {
        id: label
        x: 45
        y: 346
        color: "#f6f7f7"
        text: qsTr("SURGE")
        font.pointSize: 10
    }

    Label {
        id: label1
        x: 125
        y: 346
        color: "#fbfbfb"
        text: qsTr("SWAY")
        font.pointSize: 10
    }

    Label {
        id: label2
        x: 195
        y: 346
        color: "#f9f9f9"
        text: qsTr("HEAVE")
        font.pointSize: 10
    }

    Label {
        id: label3
        x: 276
        y: 346
        color: "#f6f7f7"
        text: qsTr("ROLL")
        font.pointSize: 10
    }

    Label {
        id: label4
        x: 347
        y: 346
        color: "#f6f7f7"
        text: qsTr("PITCH")
        font.pointSize: 10
    }

    Label {
        id: label5
        x: 428
        y: 346
        color: "#f4f4f5"
        text: qsTr("YAW")
        font.pointSize: 10
    }
}

/*##^##
Designer {
    D{i:0;formeditorZoom:0.75}
}
##^##*/
