import QtQuick 2.12
import QtQuick.Window 2.12
import QtQuick.Controls 2.12

OscillatorForm{

    //Slider signals to Cpp back-end
    tfreqSlider.onValueChanged: {
        oscillator.set_trans_freq(tfreqSlider.value)
    }
    rfreqSlider.onValueChanged: {
        oscillator.set_rot_freq(rfreqSlider.value)
    }
    surgeSlider.onValueChanged: {
        oscillator.set_surge_A(surgeSlider.value)
    }
    swaySlider.onValueChanged: {
        oscillator.set_sway_A(swaySlider.value)
    }
    heaveSlider.onValueChanged: {
        oscillator.set_heave_A((heaveSlider.value))
    }
    rollSlider.onValueChanged: {
        oscillator.set_roll_A(rollSlider.value)
    }
    pitchSlider.onValueChanged: {
        oscillator.set_pitch_A(pitchSlider.value)
    }
    yawSlider.onValueChanged: {
        oscillator.set_yaw_A(yawSlider.value)
    }
}
