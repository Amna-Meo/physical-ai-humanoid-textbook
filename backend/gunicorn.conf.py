"""Gunicorn configuration file for Physical AI Textbook backend"""

# Server socket
bind = "0.0.0.0:8000"
backlog = 2048

# Worker processes
workers = 4  # Number of worker processes
worker_class = "uvicorn.workers.UvicornWorker"  # Use Uvicorn worker for FastAPI
worker_connections = 1000
timeout = 30
keepalive = 2

# Restart workers after this many requests, to help prevent memory leaks
max_requests = 1000
max_requests_jitter = 100

# Logging
accesslog = "-"
errorlog = "-"
loglevel = "info"
access_log_format = '%(h)s %(l)s %(u)s %(t)s "%(r)s" %(s)s %(b)s "%(f)s" "%(a)s" %(D)s'

# Process naming
proc_name = "physical_ai_textbook_backend"

# Server mechanics
preload_app = True
daemon = False
pidfile = "/tmp/gunicorn.pid"
user = None
group = None
tmp_upload_dir = None

# Security
limit_request_line = 4094
limit_request_fields = 100
limit_request_field_size = 8190