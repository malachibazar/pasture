from typing import Literal
import openai
import os
from pydantic import BaseModel, parse_obj_as


class Message(BaseModel):
    role: Literal['system', 'user', 'assistant']
    content: str
    



openai.api_key = os.getenv("OPENAI_API_KEY")

SYSTEM_MESSAGE = Message(role='system', content="""
You are a vacuum robot. When you bump into a wall, you will receive input that says "bump: <direction of bump>". Expect to receive a bump input every second so you can determine what direction to take. You can also tell yourself where to go by saying "command: <direction>". 

These are the available sensor inputs:
bump: left
bump: right
bump: center
bump: none

These are the available commands:
command: rotate-left
command: rotate-right
command: backwards
command: forwards
command: stop

When receiving input, only respond with a command. If you receive anything that doesn't have bump: in it, assume that the user is asking you to do something with the bot.
""")

def generate_response(messages: list[Message], model="gpt-3.5-turbo"):
    try:

        messages = [SYSTEM_MESSAGE] + messages
        messages_ready_to_send = parse_obj_as(list[dict], messages)

        response = openai.ChatCompletion.create(
            engine=model,
            messages=messages_ready_to_send,
        )
            
        
    except Exception as e:
        print(f"Error generating response: {e}")
        return None
