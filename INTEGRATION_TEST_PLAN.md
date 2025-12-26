# Testing the Integration

## Test Plan for Backend-Frontend Integration

Now that you have both the backend deployed and the frontend on Vercel, here's how to test the integration:

## 1. Backend Health Check
Visit: `http://your-backend-server:8000/health`
Expected response: `{"status": "healthy"}`

## 2. API Endpoints Test
Test a few key API endpoints:

### Users API
- `GET /api/v1/users/me` - Should return user information (with proper authentication)
- `POST /api/v1/users/` - Should allow user creation

### AI API
- `POST /api/v1/ai/chat` - Should return AI responses
- `POST /api/v1/ai/generate` - Should generate content based on prompts

### Chapters API
- `GET /api/v1/chapters/` - Should return available chapters
- `GET /api/v1/chapters/{id}` - Should return specific chapter

### Courses API
- `GET /api/v1/courses/` - Should return available courses
- `GET /api/v1/courses/{id}` - Should return specific course

## 3. Frontend-Backend Integration Test
1. Visit your Vercel frontend URL
2. Try to access features that call backend APIs
3. Check browser console for any CORS errors
4. Verify that AI chat features work
5. Test user authentication and profile features
6. Verify that chapter/course content loads correctly

## 4. CORS Configuration Check
Make sure your backend's ALLOWED_ORIGINS includes your Vercel domain:
- In your `.env` file, check the `ALLOWED_ORIGINS` variable
- It should include your Vercel frontend URL (e.g., https://your-project.vercel.app)

## 5. Authentication Flow Test
1. Register a new user through the frontend
2. Verify the user is created in the backend
3. Login and verify JWT tokens are handled correctly
4. Test protected API endpoints

## 6. AI Features Test
1. Test the AI chat functionality
2. Verify responses are coming from the backend
3. Test different AI providers (if configured)
4. Check response quality and performance

## 7. Error Handling Test
1. Test API endpoints with invalid data
2. Verify proper error responses
3. Test with invalid authentication
4. Verify graceful error handling in the frontend

## 8. Performance Test
1. Load test the backend API endpoints
2. Verify response times are acceptable
3. Test concurrent users if applicable

## 9. Troubleshooting Common Issues

### CORS Errors
- Check that ALLOWED_ORIGINS in your backend includes your frontend domain
- Verify the origin header in your frontend requests

### API Connection Issues
- Verify the backend server is running and accessible
- Check firewall settings
- Confirm the API endpoint URLs in your frontend

### Authentication Issues
- Verify JWT configuration is consistent between frontend and backend
- Check that tokens are properly stored and sent with requests

## 10. Monitoring After Deployment
- Set up logging to monitor API usage
- Monitor error rates and response times
- Track user engagement with AI features
- Monitor database and vector store performance