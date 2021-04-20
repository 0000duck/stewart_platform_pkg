import QtQuick 2.12
import QtQuick.Controls 2.12

PageBackground {
    width: 800
    height: 480

    title: "Home"

    Rectangle {
        id: rectangle
        x: 294
        y: 140
        width: 145
        height: 45
        color: "#3a3a3a"
    }

    Label {
        id: label
        x: 201
        y: 148
        color: "#db6221"
        text: qsTr("Status:")
        font.pointSize: 15
    }

    Rectangle {
        id: rectangle1
        x: 294
        y: 73
        width: 145
        height: 45
        color: "#3a3a3a"
    }

    Label {
        id: label1
        x: 47
        y: 81
        color: "#db6221"
        text: qsTr("Power Consumption:")
        font.pointSize: 15
    }

    Rectangle {
        id: rectangle2
        x: 294
        y: 206
        width: 145
        height: 45
        color: "#3a3a3a"
    }

    Label {
        id: label2
        x: 187
        y: 214
        color: "#db6221"
        text: qsTr("Current:")
        font.pointSize: 15
    }

    Rectangle {
        id: rectangle3
        x: 294
        y: 277
        width: 145
        height: 45
        color: "#3a3a3a"
    }

    Label {
        id: label3
        x: 127
        y: 285
        color: "#db6221"
        text: qsTr("Placeholder1:")
        font.pointSize: 15
    }

    Rectangle {
        id: rectangle4
        x: 294
        y: 344
        width: 145
        height: 45
        color: "#3a3a3a"
    }

    Label {
        id: label4
        x: 127
        y: 352
        color: "#db6221"
        text: qsTr("Placeholder2:")
        font.pointSize: 15
    }
    Image {
    source: "pics/SP.png"
    width: 200
    height: 220
    x: 513
    y: 120
    }
    Rectangle {
        id: rectangle5
        x: 513
        y: 129
        width: 200
        height: 200
        color: "#ffffff"

        Label {
            id: label5
            x: 31
            y: 92
            text: qsTr("PICTURE OF PLATFORM")
        }
    }

}

/*##^##
Designer {
    D{i:0;formeditorZoom:0.66}D{i:3}D{i:4}D{i:5}D{i:6}D{i:7}D{i:8}D{i:9}D{i:10}
}
##^##*/
