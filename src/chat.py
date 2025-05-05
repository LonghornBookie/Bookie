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
import json
import subprocess

# During speech, have Bookie reconfirm "Are you looking for [Book title] by [Author]?"
# title = title_input
# author = auth_input
# 
# response = "" # to be sent to the AI prompt
# if not found:
#     
#     prompt(); # reprompt for new information | query again
# else:
#     response = ("Your book is available. Please follow me.")
#     # send data to ros2 driver

load_dotenv()

override = "#"
api_key = "#"#os.getenv("LLAMA_API_KEY")
#api_key = override


client = OpenAI(
  base_url="https://openrouter.ai/api/v1",
  api_key="api_key"
)'
'

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

messages_array = [
    {
        "role": "system",
        "content": (
            "You are Bookie, a friendly and helpful AI asistant who will asks users for the title and author of books. "
            "If you are confused, then be honest! "
            "Keep your responses short. "
            "You manage the books located at the AHG. "
            "The only books and authors that you respond with must be the most recent you've seen in message history. "
            "Your primary task is to ask users for a book located in the AHG. DO NOT DESCRIBE THE ACRONYM. "
            'After the user responds with a book, reply with a single json object like {"title": "", "author": "} and nothing else'
        )
    },
]
 
def check_book_database(response):
    if "would you like me to search our database for" in response.lower():
        isolator = response.lower().split("would you like me to search our database for")[1]
        title = isolator.split("written by")[0].strip().replace("\"","")
        author = isolator.split("written by")[1].strip().replace("\"","").replace("?","")
        # shouldn't error if this AI actually follows my commands
        return title, author
    return None, None

incoming_queue = []

def prompter(prompt):
    if len(incoming_queue) == 0:
        messages_array.append({"role": "user", "content": prompt})
        response = client.chat.completions.create(
            model="meta-llama/llama-3.2-3b-instruct:free",
            messages=messages_array
        )
        # FOR DIAGNOSING NO OUTPUT ISSUES
        print("RESPONSE:", response)
        print("CHOICES:", getattr(response, "choices", None))

        content = response.choices[0].message.content

        messages_array.append({
            "role": "assistant", 
            "content": content
        })
        return content
    most_recent_message = incoming_queue.pop()
    messages_array.append(most_recent_message)
    return most_recent_message["content"]

title_queue = []
greeted_user = False

def stream_response(prompt, output_widget):
    def run():
        try:
            global greeted_user
            print("PROMPT: " + prompt)
            response = prompter(prompt)
            print("RESPONSE: " + response)
            
            try: 
                json_obj = json.loads(response) # if this load doesn't crash, we have valid json
                messages_array.pop()
                content = "Would you like me to search our database for " + json_obj["title"] + " written by " + json_obj["author"] + "?"
                messages_array.append({
                    "role": "assistant", 
                    "content": content
                }) 
                response = content
            except Exception as ez:
                print(ez)

            tts_queue.put(response)
            speak_and_display(response, output_widget)

            title, author = check_book_database(response)
            print(title, author)
            if title and author:
                if (author[-1] == "?"):
                    author = author[:-1]
                print("IM FINDING IT" + title + " : " + author)
                result, found = db_connect.update(title.lower(), author.lower())
                something = str(found)
                print("BOOK LOCATED: " + something)
                title_queue.append(title)
                primed = True
                if found:
                    followup_msg = " Alright! I found a match. I will now take you to the book"
                    # the would you like must be followed by a yes for a correct response
                else:
                    followup_msg = "Sorry, we don\'t have that right now!"
                incoming_queue.append({
                    "role": "assistant", 
                    "content": followup_msg
                })

            if "i found a match" in response.lower():
                # use c code here
                ttl = title_queue[-1].strip().replace(" ", "_")
                print("LOCATING RIGHT NOW: " + ttl)
                subprocess.run(["ros2", "run", "nav_goals", "send_goal", ttl],cwd="~/bwi_ros2_final/")

            print(messages_array)
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

voice_button = tk.Button(input_frame, text="User Speak", command=voice_message)
voice_button.pack(side=tk.LEFT, padx=5)

entry.focus()
root.mainloop()