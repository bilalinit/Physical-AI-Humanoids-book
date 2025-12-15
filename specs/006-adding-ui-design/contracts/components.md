# Component Contracts: Style Authentication Components & Add Navbar User Integration

## Overview
This document specifies the contracts for UI components that will be styled as part of this feature. The contracts define the interfaces, props, and expected behavior of each component.

## Authentication Components

### SigninForm Component
```typescript
interface SigninFormProps {
  onSigninSuccess?: (data: any) => void;
}

interface SigninFormData {
  email: string;
  password: string;
}

interface SigninFormErrors {
  email?: string;
  password?: string;
  general?: string;
}
```

**Behavior**:
- Accepts `onSigninSuccess` callback that is called on successful authentication
- Validates email and password fields
- Calls authService.signin() with credentials
- Stores session in localStorage
- Shows loading state during authentication
- Displays error messages for validation and API errors

### SignupForm Component
```typescript
interface SignupFormProps {
  onSignupSuccess?: (data: any) => void;
}

interface SignupFormData {
  email: string;
  password: string;
  name: string;
  educationLevel?: string;
  programmingExperience?: string;
  roboticsBackground?: string;
}

interface SignupFormErrors {
  email?: string;
  password?: string;
  name?: string;
  general?: string;
}
```

**Behavior**:
- Accepts `onSignupSuccess` callback that is called on successful registration
- Validates all form fields
- Calls authService.signup() with user data
- Shows loading state during registration
- Displays error messages for validation and API errors

### Profile Component
```typescript
interface ProfileProps {}
```

**Behavior**:
- Uses useAuth hook to get user data
- Displays user information in read-only format
- Shows loading state while fetching data
- Displays error messages if profile fetch fails

## NavbarUserMenu Component
```typescript
interface NavbarUserMenuProps {}
```

**Behavior**:
- Uses useAuth hook to determine authentication state
- Shows "Sign In" link when user is not authenticated
- Shows user name/profile icon with dropdown when authenticated
- Dropdown contains "Profile" and "Sign Out" options
- Sign out clears localStorage and refreshes page

## Page Components

### SigninPage
- Uses SigninForm component
- Provides onSigninSuccess callback that redirects to homepage
- Includes logout button for testing (to be removed in final version)
- Contains link to signup page

### SignupPage
- Uses SignupForm component
- Provides onSignupSuccess callback that redirects to homepage
- Contains link to signin page

### ProfilePage
- Uses Profile component
- Shows user profile information in a styled container