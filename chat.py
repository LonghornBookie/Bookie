import os
import threading
import pyttsx3
import queue
from dotenv import load_dotenv
from openai import OpenAI
import tkinter as tk
import time
import speech_recognition as sr
import update as db_connect

# During speech, have Bookie reconfirm "Are you looking for [Book title] by [Author]?"
# title = title_input
# author = auth_input
# result, found = db_connect.update(title, author)
# response = "" # to be sent to the AI prompt
# if not found:
#     response = ("I'm sorry. That books isn't available. Would" +
#                 " you like to look for another book?")
#     prompt(); # reprompt for new information | query again
# else:
#     response = ("Your book is available. Please follow me.")
#     # send data to ros2 driver

load_dotenv()

client = OpenAI(
  base_url="https://openrouter.ai/api/v1",
  api_key="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"
)

tts_queue = queue.Queue()
engine = pyttsx3.init()
engine.setProperty('rate', 230)

def process():
    while True:
        text = tts_queue.get()
        if text is None:
            break
        engine.say(text)
        engine.runAndWait()


tts_thread = threading.Thread(target=process, daemon=True)
tts_thread.start()

def speak_and_display(text, output_widget):
    words = text.split()
    for word in words:
        output_widget.insert(tk.END, word + ' ')
        output_widget.see(tk.END)
        output_widget.update()
        time.sleep(0.1)
    output_widget.insert(tk.END, '\n')

# TODO: CREATE FUNCTIONS FOR AI AGENT
# tools = [{
#     "type": "function",
#     "name": "#",
#     "description": "#",
#     "arguments": "#"
# }]

def stream_response(prompt, output_widget):
    def run():
        try:
            stream = client.chat.completions.create(
                model="meta-llama/llama-3.2-3b-instruct:free",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "You are Bookie, a funny and helpful AI agent who assists users with book-related questions."
                            "Keep the extent of your humor to retain politeness and professionalism."
                            "If you are confused, then be honest!"
                            "Keep your responses short."
                            "You manage the books located at the AHG."
                            "Your primary task is to ask users for a book located in the AHG"
                            "For now, all books are available. " # EDIT THIS OUT
                            "If you find a match, clarify you're talking about the same book by re-stating the title and author of the book"
                            "After clarification, tell the user you're leading them to the book."
                            "For now, you DO arrive at your location" # EDIT THIS OUT
                            "Upon arriving at the location, tell the user the book is where you are currently at."
                            "If you don't find a match, tell the user you couldn't find the book. Ask the user if they're looking for anything else."
                            "If the user doesn't want to find any more books, say goodbye."
                        )
                    },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                #tools=tools,
                stream=True,
            )

            response = ""
            for event in stream:
                delta = event.choices[0].delta
                content = getattr(delta, "content", "")
                response += content

            tts_queue.put(response.strip())
            speak_and_display(response.strip(), output_widget)

        except Exception as e:
            output_widget.insert(tk.END, f"\n[FATAL] {e}\n")
            output_widget.see(tk.END)

    threading.Thread(target=run).start()

def send_message():
    prompt = entry.get()
    if not prompt.strip():
        return
    display.insert(tk.END, f"\nYou: {prompt}\n")
    entry.delete(0, tk.END)
    display.insert(tk.END, "Bookie: ")
    stream_response(prompt, display)

def voice_message():
    recognizer = sr.Recognizer()
    try:
        with sr.Microphone() as source:
            display.insert(tk.END, "\nListening...\n")
            display.see(tk.END)
            audio = recognizer.listen(source, timeout=5)
        prompt = recognizer.recognize_google(audio)
        display.insert(tk.END, f"\nYou: {prompt}\n")
        entry.delete(0, tk.END)
        display.insert(tk.END, "Bookie: ")
        stream_response(prompt,display)


    except sr.WaitTimeoutError:
        display.insert(tk.END, "\n[ERROR] No speech detected.\n")
    except sr.UnknownValueError:
        display.instert(tk.END, "\n[ERROR] Unknown audio detected.")
    except sr.RequestError as e:
        display.inster(tk.END, "\n[ERROR] STT request failed: {e}\n")

# Tkinter UI
root = tk.Tk()
root.title("Bookie Chat")
root.geometry("400x500")
root.resizable(False, False)

display = tk.Text(root, wrap=tk.WORD, bg="#f5f5f5", font=("Arial", 11))
display.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)

input_frame = tk.Frame(root)
input_frame.pack(padx=10, pady=10, fill=tk.X)

entry = tk.Entry(input_frame, font=("Arial", 11))
entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
entry.bind("<Return>", lambda event: send_message())

send_button = tk.Button(input_frame, text="Send", command=send_message)
send_button.pack(side=tk.RIGHT, padx=5)

voice_button = tk.Button(input_frame, text="Speak child", command=voice_message)
voice_button.pack(side=tk.LEFT, padx=5)

entry.focus()
root.mainloop()