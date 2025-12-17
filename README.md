📘 Physical AI Book – Interactive Learning Platform

An interactive, beginner-friendly learning platform focused on Physical AI, Robotics, and Embodied Intelligence, built using Docusaurus, FastAPI, and RAG-based AI chatbots.

This project was developed as part of a hackathon, with the goal of making advanced Physical AI concepts easy to understand, hands-on, and interactive.

🚀 Project Overview

Physical AI Book is a modern, book-like documentation website that teaches:

Physical AI foundations

Robotics & Embodied Intelligence

ROS 2 (Python)

Simulation (Gazebo, Unity, NVIDIA Isaac Sim)

Reinforcement Learning

Sim-to-Real workflows

Conversational Robotics using RAG (Retrieval-Augmented Generation)

The platform combines structured documentation with an AI-powered chatbot that answers questions directly from the book content.

🧠 Key Features
📖 Interactive Learning Book

Modular chapter-based content

Clean and readable documentation layout

Beginner-friendly explanations

Code examples and tutorials

🤖 AI Chatbot (RAG)

Built with FastAPI

Uses documentation content as knowledge base

Supports session-based conversations

Designed for future integration with:

Vector databases (Qdrant)

LLMs (Gemini)

Databases (Neon)

🎨 Modern UI

Custom Docusaurus UI (not default theme)

Book-like design

Interactive components

Clean typography and navigation

🏗️ Tech Stack
Frontend

Docusaurus

React + TypeScript

Custom CSS / UI components

Backend

FastAPI

Python

REST APIs for chatbot interaction

AI / RAG (Planned & Partially Implemented)

Gemini (LLM)

Qdrant (Vector DB)

Neon Serverless Postgres

📁 Project Structure
physical-ai-book/
├── docs/                  # Book chapters & modules
├── src/
│   ├── components/        # Custom React components (Chatbot, UI)
│   ├── pages/             # Homepage & custom pages
│   └── css/               # Custom styles
├── plan.md                # Project roadmap
├── task.md                # Development tasks
└── docusaurus.config.js

backend/
├── main.py                # FastAPI entry point
├── routes/                # API routes
├── models/                # Data models
├── rag/                   # RAG logic (stub/initial)
├── database.py
└── config.py

⚙️ How to Run Locally
1️⃣ Frontend (Docusaurus)
cd physical-ai-book
npm install
npm start


Docusaurus will run at:

http://localhost:3000

2️⃣ Backend (FastAPI)
cd backend
uvicorn backend.main:app --reload


FastAPI will run at:

http://127.0.0.1:8000


API Docs:

http://127.0.0.1:8000/docs

🔗 Frontend–Backend Integration

The chatbot UI sends questions from Docusaurus to FastAPI

FastAPI processes requests and returns responses

RAG logic currently uses placeholder responses (MVP)

Vector search and LLM integration can be added incrementally

🧪 MVP Status

✅ Frontend documentation platform
✅ FastAPI backend running
✅ Chatbot UI integrated
✅ API endpoints implemented
⏳ Full RAG pipeline (future enhancement)

🎯 Hackathon Goals

Make Physical AI learning accessible

Combine documentation + AI assistance

Demonstrate full-stack AI system design

Build an extendable learning platform

🔮 Future Improvements

Full RAG implementation with Qdrant

Gemini LLM integration

User progress tracking

Quiz & exercises

Multi-modal chatbot (images, diagrams)

Deployment (Vercel / Railway / Fly.io)

🤝 Contributors

Muhammad Usaid Khan – Full-stack Developer

📜 License

This project is open-source and created for educational and hackathon purposes.
