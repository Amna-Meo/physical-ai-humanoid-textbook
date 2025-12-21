from typing import Optional
from sqlalchemy.orm import Session
from passlib.context import CryptContext
from datetime import datetime, timedelta
from jose import JWTError, jwt
import os
from ..models.user import User, UserCreate, UserUpdate
from ..lib.database import get_db

# Password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# JWT settings
SECRET_KEY = os.getenv("SECRET_KEY", "your-secret-key-here")
ALGORITHM = os.getenv("ALGORITHM", "HS256")
ACCESS_TOKEN_EXPIRE_MINUTES = int(os.getenv("ACCESS_TOKEN_EXPIRE_MINUTES", "30"))


class UserService:
    def __init__(self, db: Session):
        self.db = db

    def verify_password(self, plain_password: str, hashed_password: str) -> bool:
        """
        Verify a plain password against a hashed password
        """
        return pwd_context.verify(plain_password, hashed_password)

    def get_password_hash(self, password: str) -> str:
        """
        Hash a password
        """
        return pwd_context.hash(password)

    def get_user_by_email(self, email: str) -> Optional[User]:
        """
        Get a user by email
        """
        return self.db.query(User).filter(User.email == email).first()

    def get_user_by_username(self, username: str) -> Optional[User]:
        """
        Get a user by username
        """
        return self.db.query(User).filter(User.username == username).first()

    def get_user_by_id(self, user_id: int) -> Optional[User]:
        """
        Get a user by ID
        """
        return self.db.query(User).filter(User.id == user_id).first()

    def create_user(self, user_create: UserCreate) -> User:
        """
        Create a new user
        """
        # Check if user with email already exists
        existing_user = self.get_user_by_email(user_create.email)
        if existing_user:
            raise ValueError("Email already registered")

        # Check if user with username already exists
        existing_user = self.get_user_by_username(user_create.username)
        if existing_user:
            raise ValueError("Username already taken")

        # Hash the password
        hashed_password = self.get_password_hash(user_create.password)

        # Create the user object
        db_user = User(
            email=user_create.email,
            username=user_create.username,
            full_name=user_create.full_name,
            hashed_password=hashed_password,
            professional_background=user_create.professional_background,
            learning_preferences=user_create.learning_preferences,
            timezone=user_create.timezone,
            language_preference=user_create.language_preference
        )

        # Add to database
        self.db.add(db_user)
        self.db.commit()
        self.db.refresh(db_user)

        return db_user

    def update_user(self, user_id: int, user_update: UserUpdate) -> Optional[User]:
        """
        Update user information
        """
        db_user = self.get_user_by_id(user_id)
        if not db_user:
            return None

        # Update fields if provided
        if user_update.full_name is not None:
            db_user.full_name = user_update.full_name
        if user_update.professional_background is not None:
            db_user.professional_background = user_update.professional_background
        if user_update.learning_preferences is not None:
            db_user.learning_preferences = user_update.learning_preferences
        if user_update.timezone is not None:
            db_user.timezone = user_update.timezone
        if user_update.language_preference is not None:
            db_user.language_preference = user_update.language_preference

        self.db.commit()
        self.db.refresh(db_user)

        return db_user

    def authenticate_user(self, username: str, password: str) -> Optional[User]:
        """
        Authenticate a user by username/email and password
        """
        # Try to find user by email first, then by username
        db_user = self.get_user_by_email(username)
        if not db_user:
            db_user = self.get_user_by_username(username)

        if not db_user or not self.verify_password(password, db_user.hashed_password):
            return None

        return db_user

    def create_access_token(self, data: dict, expires_delta: Optional[timedelta] = None):
        """
        Create a JWT access token
        """
        to_encode = data.copy()
        if expires_delta:
            expire = datetime.utcnow() + expires_delta
        else:
            expire = datetime.utcnow() + timedelta(minutes=15)
        to_encode.update({"exp": expire})
        encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
        return encoded_jwt

    def verify_token(self, token: str) -> Optional[dict]:
        """
        Verify a JWT token and return the payload
        """
        try:
            payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
            return payload
        except JWTError:
            return None

    def deactivate_user(self, user_id: int) -> bool:
        """
        Deactivate a user account
        """
        db_user = self.get_user_by_id(user_id)
        if not db_user:
            return False

        db_user.is_active = False
        self.db.commit()
        return True

    def activate_user(self, user_id: int) -> bool:
        """
        Activate a user account
        """
        db_user = self.get_user_by_id(user_id)
        if not db_user:
            return False

        db_user.is_active = True
        self.db.commit()
        return True


# Convenience function to get user service
def get_user_service(db: Session) -> UserService:
    return UserService(db)