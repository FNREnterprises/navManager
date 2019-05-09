
import speech_recognition as sr
import pyttsx3

import config

engine = None



def listenForCommand(recognizer, microphone):

    # adjust the recognizer sensitivity to ambient noise and record audio
    # from the microphone
    with microphone as source:
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source, timeout=5.0)

    # set up the response object
    response = {
        "success": True,
        "error": None,
        "transcription": None
    }

    # try recognizing the speech in the recording
    # if a RequestError or UnknownValueError exception is caught,
    #     update the response object accordingly
    #recognizer.Timeouts.EndSilenceTimeout = TimeSpan.FromSeconds(1.2)
    try:
        response["transcription"] = recognizer.recognize_google(audio)
    except sr.RequestError:
        # API was unreachable or unresponsive
        response["success"] = False
        response["error"] = "API unavailable"
    except sr.UnknownValueError:
        # speech was unintelligible
        response["error"] = "Unable to recognize speech"

    return response


def reply(msg):
    print(msg)
    engine.say(msg)
    engine.runAndWait()


def startListening():

    if not config.micOn:

        engine = pyttsx3

        voiceId = "HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\Speech\Voices\Tokens\TTS_MS_EN-US_DAVID_11.0"
        engine.setProperty('voice', voiceId)
        rate = engine.getProperty('rate')
        engine.setProperty('rate', rate-30)

        # create recognizer and mic instances
        recognizer = sr.Recognizer()
        microphone = sr.Microphone()

        # check that recognizer and microphone arguments are appropriate type
        if not isinstance(recognizer, sr.Recognizer):
            raise TypeError("`recognizer` must be `Recognizer` instance")

        if not isinstance(microphone, sr.Microphone):
            raise TypeError("`microphone` must be `Microphone` instance")

    while True:

        result = listenForCommand(recognizer, microphone)

        print(result)

