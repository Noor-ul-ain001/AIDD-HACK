FROM python:3.11-slim

WORKDIR /app

# Install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application
COPY hf_spaces.py .
COPY src/ ./src/
COPY .env.example .

# Expose port
EXPOSE 7860

# Run the app
CMD ["python", "hf_spaces.py"]
