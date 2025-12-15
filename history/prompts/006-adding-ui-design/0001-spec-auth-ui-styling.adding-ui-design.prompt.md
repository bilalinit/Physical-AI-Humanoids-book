---
id: 0001
title: "Spec Auth UI Styling"
stage: spec
date_iso: "2025-12-15"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "adding-ui-design"
branch: "006-adding-ui-design"
user: "user"
command: "/sp.specify"
labels: ["authentication", "ui", "styling", "docusaurus"]
links:
  spec: "specs/006-adding-ui-design/spec.md"
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - specs/006-adding-ui-design/spec.md
  - specs/006-adding-ui-design/checklists/requirements.md
  - history/prompts/006-adding-ui-design/0001-spec-auth-ui-styling.adding-ui-design.prompt.md
tests_yaml: |
  - "Specification completeness validation"
  - "Requirements checklist validation"
---

## Prompt

```
we are now moving on to the next phase, name the new branch "006-adding-ui-design" the specs: # Style Authentication Components & Add Navbar User Integration

**Subject:** Style Authentication Components & Add Navbar User Integration (Preserve Logic)

## Context

I have authentication components in my Docusaurus project:

**Form Components** (`frontend/src/components/Auth/`):
- `SigninForm.tsx` - email/password login form
- `SignupForm.tsx` - registration form with name, email, password, and 3 dropdown selects (education level, programming experience, robotics background)
- `Profile.tsx` - read-only user profile display showing name, email, education level, programming experience, robotics background, and member since date

**Page Wrappers** (`frontend/src/pages/`):
- `signin.tsx` - wraps SigninForm, includes a logout button
- `signup.tsx` - wraps SignupForm
- `profile.tsx` - wraps Profile component

**Auth Hook** (`frontend/src/hooks/useAuth.tsx`) - provides `user`, `loading`, and `error` state

The components have working TypeScript logic but use inline styles. I need professional styling that matches the Docusaurus Infima theme.

## Crucial Constraint - Functional Integrity

- **DO NOT modify** existing hooks (`useState`, `useEffect`), event handlers (`onSubmit`, `onChange`), or API calls
- **DO NOT refactor** the code structure (wrapping in containers is allowed)
- **Your task is strictly presentation + new UI elements below**

---

## Part 1: CSS Module Styling

### Technical Requirements
1. **CSS Approach:** Use CSS Modules (`.module.css` files)
2. **Docusaurus Infima Variables:** Use `var(--ifm-color-primary)`, `var(--ifm-background-color)`, `var(--ifm-font-color-base)`, etc. for Dark Mode support
3. **Responsive Design:** Mobile-friendly layouts

### Components to Style

**SigninForm/SignupForm:**
- Card-style container with subtle shadow/border
- Form labels, text inputs, and `<select>` dropdowns (focus states, borders)
- Error message styling (`.error-message`, `.error` classes already exist)
- Submit buttons (replace inline styles with proper classes)

**Profile:**
- Card layout for user profile info
- Styled profile field labels and values in a clean grid or list format
- Loading and error state styling

**Page Wrappers:**
- Style the logout button in `signin.tsx`

---

## Part 2: Navigation Between Sign In & Sign Up

Add navigation links to switch between the authentication pages:
- On **SigninForm**: Add "Don't have an account? [Sign Up](/signup)" link below the form
- On **SignupForm**: Add "Already have an account? [Sign In](/signin)" link below the form

Use Docusaurus `<Link>` component (`@docusaurus/Link`) for client-side navigation.

---

## Part 3: Navbar User Profile Integration

Create a **custom navbar item** that displays authentication state:

**When user is NOT logged in:**
- Show a "Sign In" link in the navbar (right side, before GitHub link)

**When user IS logged in:**
- Show user's name or a user icon
- On click/hover: dropdown menu with:
  - "Profile" link → `/profile`
  - "Sign Out" button (clears `localStorage.removeItem('auth_session')` and reloads)

**Implementation approach:**
1. Create a new component `frontend/src/components/NavbarUserMenu/index.tsx`
2. Use the `useAuth` hook to get the current user state
3. Swizzle or extend the Docusaurus navbar using `theme/NavbarItems/ComponentTypes.tsx` or add to `docusaurus.config.ts` navbar items as a custom component

---

## Design Aesthetic

- Clean, minimalist, accessible
- Hover/focus effects on buttons and inputs
- Generous spacing/padding
- Error states should be visually distinct
- Navbar dropdown should match Docusaurus theme styling
```

## Response

Created specification for styling authentication components and adding navbar user integration. The specification includes:

1. User scenarios with priorities (P1-P3) for styled forms, navigation, and navbar integration
2. Functional requirements (FR-001 to FR-020) covering CSS modules, responsive design, navigation links, and navbar integration
3. Key entities including authentication components, NavbarUserMenu, CSS Modules, and Docusaurus Infima theme
4. Success criteria (SC-001 to SC-008) with measurable outcomes for styling, responsiveness, and functionality

The specification maintains functional integrity by preserving existing authentication logic while focusing on presentation improvements. All requirements are testable and technology-agnostic where appropriate.

A validation checklist was also created to ensure specification quality.