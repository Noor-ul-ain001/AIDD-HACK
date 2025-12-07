from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
from .chat import router as chat_router
from .auth import router as auth_router
from .chapters import router as chapters_router
from .ingestion import router as ingestion_router # Import the ingestion router

load_dotenv()

app = FastAPI()

# Set up CORS
origins = [
    "*",  # In a production environment, you should restrict this to your frontend's domain
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(chat_router)
app.include_router(auth_router, prefix="/auth", tags=["auth"])
app.include_router(chapters_router)
app.include_router(ingestion_router, prefix="/ingestion", tags=["ingestion"]) # Include the ingestion router

@app.get("/")
async def read_root():
    return {"message": "Welcome to the Physical AI & Humanoid Robotics API!"}
