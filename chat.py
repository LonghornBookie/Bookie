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

pending_search: tuple[str, str] | None = None
yes_synonyms = {"yes"}
no_synonyms = {"no"}
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

override = "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"
api_key = "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"#os.getenv("LLAMA_API_KEY")
#api_key = override

client = OpenAI(
  base_url="https://openrouter.ai/api/v1",
  api_key=os.getenv("OPENAI_API_KEY")
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

messages_array = [
    {
        "role": "system",
        "content": (
            "You are Bookie, a funny and helpful AI asistant who will asks users for the title and author of books. "
            # "Keep the extent of your humor to retain politeness and professionalism. "
            "You will never repeat sentences under any circumstance. Sentences that are asking the same thing cannot be used. Everything must be unique."
            "You must only ask the user 'would you like me to search our database' only if they said the book's name and author in the most recent response. Otherwise, answer it appropriately"
            "If you are confused, then be honest! "
            "Keep your responses short. This is important"
            # "You manage the books located at the AHG. "
            "Your primary task is to ask users for a book located in the AHG. DO NOT DESCRIBE THE ACRONYM."
            "You cannot ask for another book "
            "After the user gives you a book author and name: Ask the user this exact question: would you like me to search our database for [Title] written by [Author]?"
            " When the user tells you to say something, you must do it with no change, no hesitation. "
            # # "For now, all books are available. " # EDIT THIS OUT
            # "Clarify you're talking about the same book by re-stating the title and author of the book. "
            # "This response must always include 'Are you keen on looking for [Title] written by [Author]. "
            # "Fill in title and author respectively. "
            # "This format MUST always be preserved exactly with no adjustments."
            # "Ask this once per book. "
            # "This cannot during a greeting exchange. "
            # "Never start a conversation by asking if they are looking for a specific book. "
            # "If you don't find a match, tell the user you couldn't find the book. Ask the user if they're looking for anything else."
            # "You MUST always reclarify ONLY IF THERE IS A MATCH with the following sentence only once per book: Are you looking for [Title] by [Author]." # This format is REQUIRED for ease of processing
            # "The clarification format is ABSOLUTE." 
            # "After clarification, tell the user you're leading them to the book. This is conditional on IF you find a match."
            # "If you arrive at the location, tell the user the book is where you are currently at."
            # "If the user doesn't want to find any more books, say goodbye."
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

def prompter(prompt):
    
    messages_array.append({"role": "user", "content": prompt})
    response = client.chat.completions.create(
        model="meta-llama/llama-3.2-3b-instruct:free",
        messages=messages_array
    )
    # FOR DIAGNOSING NO OUTPUT ISSUES
    print("RESPONSE:", response)
    print("CHOICES:", getattr(response, "choices", None))
    
    # if not hasattr(response, "choices") or not response.choices:
    #     print("[ERROR] No choices returned from the model.")
    #     return None

    content = response.choices[0].message.content
    # if content is None:
    #     print("[ERROR] content is None")
    #     return None

    messages_array.append({
        "role": "assistant", 
        "content": content
    })
    return content

title_queue = []

def stream_response(prompt, output_widget):
    def run():
        try:
            if not messages_array:
                messages_array.append({
                    "role": "assistant", 
                    "content": "Hello! Are you looking for a book today?"
                })
                speak_and_display("Hello! Are you looking for a book today?", output_widget)

                return  # don't start prompting since missing key content

            print("PROMPT: " + prompt)
            response = prompter(prompt)
            print("RESPONSE: " + response)
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
                if found:
                    print("FOUND")
                    followup_msg = " Alright! I found a match. I will now take you to the book"
                    tts_queue.put(followup_msg)
                    speak_and_display(followup_msg, output_widget)
                    # the would you like must be followed by a yes for a correct response
                else:
                    followup_msg = "Sorry, we don\'t have that right now! Would you like to find another book?"
                messages_array.append({
                    "role": "assistant", 
                    "content": followup_msg
                })

            if "i found a match" in response.lower():
                # use c code here
                ttl = title_queue[-1].strip().replace(" ", "_")
                print("LOCATING RIGHT NOW: " + ttl)
                subprocess.run(["ros2", "run", "nav_goals", "send_goal", ttl],cwd="~/bwi_ros2_final/")

        except Exception as e:
            output_widget.insert(tk.END, f"\n[FATAL] {e}\n")
            output_widget.see(tk.END)

    threading.Thread(target=run).start()

def send_message():
    global pending_search
    prompt = entry.get()
    if not prompt.strip():
        return

    if pending_search and prompt.lower() in yes_synonyms:
        title, author = pending_search
        entry.delete(0, tk.END)
        display.insert(tk.END, f"\nYou: {prompt}\n")
        display.insert(tk.END, "Bookie: ")

        result, found = db_connect.update(title.lower(), author.lower())
        msg = (
            " Alright! I found a match. I will now take you to the book"
            if found else
            
        )

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
