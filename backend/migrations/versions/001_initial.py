"""Initial migration for Physical AI & Humanoid Robotics Textbook

Revision ID: 001_initial
Revises:
Create Date: 2025-12-17 14:30:00.000000

"""
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# revision identifiers
revision = '001_initial'
down_revision = None
branch_labels = None
depends_on = None


def upgrade() -> None:
    # Create users table
    op.create_table('users',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('email', sa.String(), nullable=False),
        sa.Column('username', sa.String(), nullable=False),
        sa.Column('full_name', sa.String(), nullable=True),
        sa.Column('hashed_password', sa.String(), nullable=False),
        sa.Column('is_active', sa.Boolean(), nullable=True),
        sa.Column('is_verified', sa.Boolean(), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.Column('updated_at', sa.DateTime(timezone=True), nullable=True),
        sa.Column('professional_background', sa.Text(), nullable=True),
        sa.Column('learning_preferences', sa.Text(), nullable=True),
        sa.Column('timezone', sa.String(), nullable=True),
        sa.Column('language_preference', sa.String(), nullable=True),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_users_email'), 'users', ['email'], unique=True)
    op.create_index(op.f('ix_users_id'), 'users', ['id'], unique=False)
    op.create_index(op.f('ix_users_username'), 'users', ['username'], unique=True)

    # Create chapters table
    op.create_table('chapters',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('title', sa.String(), nullable=False),
        sa.Column('slug', sa.String(), nullable=False),
        sa.Column('content', sa.Text(), nullable=False),
        sa.Column('description', sa.Text(), nullable=True),
        sa.Column('chapter_number', sa.Integer(), nullable=False),
        sa.Column('word_count', sa.Integer(), nullable=True),
        sa.Column('reading_time_minutes', sa.Integer(), nullable=True),
        sa.Column('chapter_metadata', postgresql.JSON(astext_type=sa.Text()), nullable=True),
        sa.Column('citations', postgresql.JSON(astext_type=sa.Text()), nullable=True),
        sa.Column('is_published', sa.Boolean(), nullable=True),
        sa.Column('is_ai_optimized', sa.Boolean(), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.Column('updated_at', sa.DateTime(timezone=True), nullable=True),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_chapters_id'), 'chapters', ['id'], unique=False)
    op.create_index(op.f('ix_chapters_slug'), 'chapters', ['slug'], unique=True)

    # Create chapter_content_blocks table
    op.create_table('chapter_content_blocks',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('chapter_id', sa.Integer(), nullable=False),
        sa.Column('block_type', sa.String(), nullable=False),
        sa.Column('content', sa.Text(), nullable=False),
        sa.Column('order_index', sa.Integer(), nullable=False),
        sa.Column('block_metadata', postgresql.JSON(astext_type=sa.Text()), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.ForeignKeyConstraint(['chapter_id'], ['chapters.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_chapter_content_blocks_id'), 'chapter_content_blocks', ['id'], unique=False)

    # Create learning_paths table
    op.create_table('learning_paths',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('title', sa.String(), nullable=False),
        sa.Column('slug', sa.String(), nullable=False),
        sa.Column('description', sa.Text(), nullable=True),
        sa.Column('is_active', sa.Boolean(), nullable=True),
        sa.Column('is_personalized', sa.Boolean(), nullable=True),
        sa.Column('difficulty_level', sa.String(), nullable=True),
        sa.Column('estimated_duration_hours', sa.Integer(), nullable=True),
        sa.Column('path_metadata', postgresql.JSON(astext_type=sa.Text()), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.Column('updated_at', sa.DateTime(timezone=True), nullable=True),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_learning_paths_id'), 'learning_paths', ['id'], unique=False)
    op.create_index(op.f('ix_learning_paths_slug'), 'learning_paths', ['slug'], unique=True)

    # Create learning_path_steps table
    op.create_table('learning_path_steps',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('learning_path_id', sa.Integer(), nullable=False),
        sa.Column('step_number', sa.Integer(), nullable=False),
        sa.Column('title', sa.String(), nullable=False),
        sa.Column('description', sa.Text(), nullable=True),
        sa.Column('content_type', sa.String(), nullable=False),
        sa.Column('content_id', sa.Integer(), nullable=False),
        sa.Column('required', sa.Boolean(), nullable=True),
        sa.Column('estimated_duration_minutes', sa.Integer(), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.ForeignKeyConstraint(['learning_path_id'], ['learning_paths.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_learning_path_steps_id'), 'learning_path_steps', ['id'], unique=False)

    # Create user_learning_paths table
    op.create_table('user_learning_paths',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('user_id', sa.Integer(), nullable=False),
        sa.Column('learning_path_id', sa.Integer(), nullable=False),
        sa.Column('status', sa.String(), nullable=True),
        sa.Column('current_step', sa.Integer(), nullable=True),
        sa.Column('progress_percentage', sa.Integer(), nullable=True),
        sa.Column('started_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.Column('completed_at', sa.DateTime(timezone=True), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.Column('updated_at', sa.DateTime(timezone=True), nullable=True),
        sa.ForeignKeyConstraint(['learning_path_id'], ['learning_paths.id'], ),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_user_learning_paths_id'), 'user_learning_paths', ['id'], unique=False)

    # Create user_learning_path_progress table
    op.create_table('user_learning_path_progress',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('user_id', sa.Integer(), nullable=False),
        sa.Column('learning_path_id', sa.Integer(), nullable=False),
        sa.Column('step_id', sa.Integer(), nullable=False),
        sa.Column('status', sa.String(), nullable=True),
        sa.Column('score', sa.Integer(), nullable=True),
        sa.Column('completed_at', sa.DateTime(timezone=True), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.Column('updated_at', sa.DateTime(timezone=True), nullable=True),
        sa.ForeignKeyConstraint(['learning_path_id'], ['learning_paths.id'], ),
        sa.ForeignKeyConstraint(['step_id'], ['learning_path_steps.id'], ),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_user_learning_path_progress_id'), 'user_learning_path_progress', ['id'], unique=False)

    # Create ai_chat_sessions table
    op.create_table('ai_chat_sessions',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('user_id', sa.Integer(), nullable=True),
        sa.Column('session_title', sa.String(), nullable=True),
        sa.Column('is_active', sa.Boolean(), nullable=True),
        sa.Column('session_metadata', postgresql.JSON(astext_type=sa.Text()), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.Column('updated_at', sa.DateTime(timezone=True), nullable=True),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_ai_chat_sessions_id'), 'ai_chat_sessions', ['id'], unique=False)

    # Create ai_chat_messages table
    op.create_table('ai_chat_messages',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('session_id', sa.Integer(), nullable=False),
        sa.Column('user_id', sa.Integer(), nullable=True),
        sa.Column('message_type', sa.String(), nullable=False),
        sa.Column('content', sa.Text(), nullable=False),
        sa.Column('role', sa.String(), nullable=False),
        sa.Column('sources', postgresql.JSON(astext_type=sa.Text()), nullable=True),
        sa.Column('context_chapters', postgresql.JSON(astext_type=sa.Text()), nullable=True),
        sa.Column('timestamp', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.Column('is_grounding_validated', sa.Boolean(), nullable=True),
        sa.ForeignKeyConstraint(['session_id'], ['ai_chat_sessions.id'], ),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_ai_chat_messages_id'), 'ai_chat_messages', ['id'], unique=False)

    # Create ai_interaction_logs table
    op.create_table('ai_interaction_logs',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('user_id', sa.Integer(), nullable=True),
        sa.Column('session_id', sa.Integer(), nullable=False),
        sa.Column('interaction_type', sa.String(), nullable=False),
        sa.Column('input_text', sa.Text(), nullable=False),
        sa.Column('output_text', sa.Text(), nullable=False),
        sa.Column('ai_model_used', sa.String(), nullable=False),
        sa.Column('tokens_input', sa.Integer(), nullable=True),
        sa.Column('tokens_output', sa.Integer(), nullable=True),
        sa.Column('response_time_ms', sa.Integer(), nullable=True),
        sa.Column('is_accurate', sa.Boolean(), nullable=True),
        sa.Column('feedback_score', sa.Integer(), nullable=True),
        sa.Column('feedback_text', sa.Text(), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.ForeignKeyConstraint(['session_id'], ['ai_chat_sessions.id'], ),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_ai_interaction_logs_id'), 'ai_interaction_logs', ['id'], unique=False)

    # Create ai_preferences table
    op.create_table('ai_preferences',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('user_id', sa.Integer(), nullable=False),
        sa.Column('preference_key', sa.String(), nullable=False),
        sa.Column('preference_value', sa.String(), nullable=False),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.Column('updated_at', sa.DateTime(timezone=True), nullable=True),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_ai_preferences_id'), 'ai_preferences', ['id'], unique=False)

    # Create citations table
    op.create_table('citations',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('citation_key', sa.String(), nullable=False),
        sa.Column('title', sa.String(), nullable=False),
        sa.Column('authors', sa.Text(), nullable=True),
        sa.Column('journal', sa.String(), nullable=True),
        sa.Column('volume', sa.String(), nullable=True),
        sa.Column('issue', sa.String(), nullable=True),
        sa.Column('pages', sa.String(), nullable=True),
        sa.Column('year', sa.Integer(), nullable=True),
        sa.Column('publisher', sa.String(), nullable=True),
        sa.Column('doi', sa.String(), nullable=True),
        sa.Column('url', sa.String(), nullable=True),
        sa.Column('citation_type', sa.String(), nullable=False),
        sa.Column('raw_citation', sa.Text(), nullable=False),
        sa.Column('apa_formatted', sa.Text(), nullable=False),
        sa.Column('bibtex_formatted', sa.Text(), nullable=True),
        sa.Column('is_verified', sa.Boolean(), nullable=True),
        sa.Column('citation_metadata', postgresql.JSON(astext_type=sa.Text()), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.Column('updated_at', sa.DateTime(timezone=True), nullable=True),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_citations_citation_key'), 'citations', ['citation_key'], unique=True)
    op.create_index(op.f('ix_citations_id'), 'citations', ['id'], unique=False)

    # Create chapter_citations table
    op.create_table('chapter_citations',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('chapter_id', sa.Integer(), nullable=False),
        sa.Column('citation_id', sa.Integer(), nullable=False),
        sa.Column('citation_context', sa.Text(), nullable=True),
        sa.Column('page_reference', sa.String(), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.ForeignKeyConstraint(['chapter_id'], ['chapters.id'], ),
        sa.ForeignKeyConstraint(['citation_id'], ['citations.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_chapter_citations_id'), 'chapter_citations', ['id'], unique=False)

    # Create citation_verifications table
    op.create_table('citation_verifications',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('citation_id', sa.Integer(), nullable=False),
        sa.Column('verified_by', sa.Integer(), nullable=True),
        sa.Column('verification_method', sa.String(), nullable=False),
        sa.Column('is_accurate', sa.Boolean(), nullable=False),
        sa.Column('notes', sa.Text(), nullable=True),
        sa.Column('verified_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.ForeignKeyConstraint(['citation_id'], ['citations.id'], ),
        sa.ForeignKeyConstraint(['verified_by'], ['users.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_citation_verifications_id'), 'citation_verifications', ['id'], unique=False)


def downgrade() -> None:
    op.drop_index(op.f('ix_citation_verifications_id'), table_name='citation_verifications')
    op.drop_table('citation_verifications')
    op.drop_index(op.f('ix_chapter_citations_id'), table_name='chapter_citations')
    op.drop_table('chapter_citations')
    op.drop_index(op.f('ix_citations_id'), table_name='citations')
    op.drop_index(op.f('ix_citations_citation_key'), table_name='citations')
    op.drop_table('citations')
    op.drop_index(op.f('ix_ai_preferences_id'), table_name='ai_preferences')
    op.drop_table('ai_preferences')
    op.drop_index(op.f('ix_ai_interaction_logs_id'), table_name='ai_interaction_logs')
    op.drop_table('ai_interaction_logs')
    op.drop_index(op.f('ix_ai_chat_messages_id'), table_name='ai_chat_messages')
    op.drop_table('ai_chat_messages')
    op.drop_index(op.f('ix_ai_chat_sessions_id'), table_name='ai_chat_sessions')
    op.drop_table('ai_chat_sessions')
    op.drop_index(op.f('ix_user_learning_path_progress_id'), table_name='user_learning_path_progress')
    op.drop_table('user_learning_path_progress')
    op.drop_index(op.f('ix_user_learning_paths_id'), table_name='user_learning_paths')
    op.drop_table('user_learning_paths')
    op.drop_index(op.f('ix_learning_path_steps_id'), table_name='learning_path_steps')
    op.drop_table('learning_path_steps')
    op.drop_index(op.f('ix_learning_paths_slug'), table_name='learning_paths')
    op.drop_index(op.f('ix_learning_paths_id'), table_name='learning_paths')
    op.drop_table('learning_paths')
    op.drop_index(op.f('ix_chapter_content_blocks_id'), table_name='chapter_content_blocks')
    op.drop_table('chapter_content_blocks')
    op.drop_index(op.f('ix_chapters_slug'), table_name='chapters')
    op.drop_index(op.f('ix_chapters_id'), table_name='chapters')
    op.drop_table('chapters')
    op.drop_index(op.f('ix_users_username'), table_name='users')
    op.drop_index(op.f('ix_users_id'), table_name='users')
    op.drop_index(op.f('ix_users_email'), table_name='users')
    op.drop_table('users')