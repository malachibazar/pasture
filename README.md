# Roomba Controller (Work in Progress)

Hopefully by the end of this project, the roomba will be using sonar sensor data to map itself in a room.


## End Goal
The end goal of this project is to have a robot that can take verbal commands/conversation to move through a space. I'm going to use the OpenAI API to translate the commands into movement.

### Milestones
1. Map and navigate a space dynamically
1. Build a map of multiple rooms and navigate between rooms
1. Make a "cleaning" mode that paths out the rooms and cleans the whole foor
1. Make all of the end methods and functions available to OpenAI API
    - What I'm thinking this will look like is filling in the system message of the ChatGPT API with something that gives a breakdown of commands available for the AI to send.