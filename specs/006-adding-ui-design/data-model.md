# Data Model: Style Authentication Components & Add Navbar User Integration

## Authentication Components

### SigninForm Component
- **Props**: `onSigninSuccess` (callback function)
- **State**:
  - `formData` (email, password)
  - `errors` (field-specific and general)
  - `loading` (boolean)
- **UI Elements**:
  - Form container with card-style layout
  - Email input field with validation
  - Password input field with validation
  - Submit button with loading state
  - Error message display
  - Navigation link to signup page

### SignupForm Component
- **Props**: `onSignupSuccess` (callback function)
- **State**:
  - `formData` (email, password, name, educationLevel, programmingExperience, roboticsBackground)
  - `errors` (field-specific and general)
  - `loading` (boolean)
- **UI Elements**:
  - Form container with card-style layout
  - Name input field with validation
  - Email input field with validation
  - Password input field with validation
  - Education level dropdown
  - Programming experience dropdown
  - Robotics background dropdown
  - Submit button with loading state
  - Error message display
  - Navigation link to signin page

### Profile Component
- **Props**: None
- **State**:
  - User data from auth context
  - Loading state
  - Error state
- **UI Elements**:
  - Profile container with card-style layout
  - User information display (name, email, education level, etc.)
  - Loading indicator
  - Error message display

## NavbarUserMenu Component

### State Management
- User authentication status (from useAuth hook)
- Dropdown menu open/close state
- Loading state for auth operations

### UI Elements
- **Unauthenticated State**:
  - "Sign In" link in navbar
- **Authenticated State**:
  - User name or profile icon as dropdown trigger
  - Dropdown menu with:
    - Profile link
    - Sign out button

### Event Handlers
- Sign out functionality (clears localStorage and refreshes page)
- Dropdown toggle functionality
- Navigation to profile page

## CSS Modules

### Styling Approach
- Use Docusaurus Infima CSS variables for consistent theming
- Implement responsive design for mobile, tablet, and desktop
- Apply card-style containers with subtle shadows/borders
- Include proper spacing and padding for accessibility
- Add hover/focus states for interactive elements
- Style error states to be visually distinct

### Theme Variables to Use
- `--ifm-color-primary` for primary buttons and accents
- `--ifm-color-emphasis-*` for borders and emphasis
- `--ifm-spacing-*` for consistent spacing
- `--ifm-font-size-*` for typography consistency
- `--ifm-global-shadow` for card shadows
- Dark mode support through Infima's dark theme variables