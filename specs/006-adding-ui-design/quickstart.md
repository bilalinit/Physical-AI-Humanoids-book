# Quickstart Guide: Style Authentication Components & Add Navbar User Integration

## Overview
This guide provides a quick setup for implementing styled authentication components and navbar user integration in the Docusaurus site. The implementation replaces inline styles with CSS Modules using Docusaurus Infima theme variables.

## Prerequisites
- Node.js 18+ and npm/yarn
- Docusaurus project already set up
- Existing authentication infrastructure (Better Auth, useAuth hook)

## Files to Create/Update

### 1. CSS Module Files (for authentication components)
- `frontend/src/components/Auth/SigninForm.module.css` - Styles for sign-in form
- `frontend/src/components/Auth/SignupForm.module.css` - Styles for sign-up form
- `frontend/src/components/Auth/Profile.module.css` - Styles for profile component

### 2. Navbar User Menu Component
- `frontend/src/components/NavbarUserMenu/NavbarUserMenu.tsx` - New component
- `frontend/src/components/NavbarUserMenu/NavbarUserMenu.module.css` - New styles

### 3. Profile Page
- `frontend/src/pages/profile.tsx` - New page for user profile

### 4. Update Existing Files
- `frontend/src/pages/signin.tsx` - Add navigation link to signup
- `frontend/src/pages/signup.tsx` - Add navigation link to signin
- `docusaurus.config.ts` - Add NavbarUserMenu to navbar items

## Implementation Steps

### Step 1: Create CSS Modules for Auth Components
Replace inline styles in authentication components with CSS Modules that use Infima variables for consistent theming.

### Step 2: Create Navbar User Menu
Create a new component that integrates with Docusaurus navbar and displays authentication state with dropdown menu for authenticated users.

### Step 3: Add Navigation Links
Add links between sign-in and sign-up pages using Docusaurus `<Link>` component for client-side navigation.

### Step 4: Create Profile Page
Create a new profile page that displays user information using the existing Profile component.

## Key Features
- Responsive design that works on mobile, tablet, and desktop
- Consistent styling using Docusaurus Infima theme variables
- Preserved authentication logic (no changes to hooks or API calls)
- Card-style containers with proper spacing and shadows
- Accessible error states and interactive elements
- Dark mode support through Infima theme

## Testing
1. Visit `/signin` and `/signup` pages to verify styled forms
2. Test navigation between auth pages
3. Verify navbar displays correct content based on auth state
4. Test dropdown functionality when logged in
5. Verify profile page displays user information correctly