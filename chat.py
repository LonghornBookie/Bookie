import os
import requests
import threading
from dotenv import load_dotenv
from openai import OpenAI
import tkinter as tk
import time

load_dotenv()

client = OpenAI(
  base_url="https://openrouter.ai/api/v1",
  api_key=os.getenv("LLAMA_API_KEY")
)

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
                            "You are Bookie, a funny and helpful AI agent who assists users with book-related question."
                            "Keep the extent of your humor to retain politeness and professionalism."
                            "If you are confused, then be honest!"
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
                output_widget.insert(tk.END, content)
                output_widget.see(tk.END)
                output_widget.update()
                time.sleep(0.05)
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

entry.focus()
root.mainloop()