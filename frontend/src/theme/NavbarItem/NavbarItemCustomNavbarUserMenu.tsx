import React from 'react';
import { AuthProvider } from '@site/src/hooks/useAuth';
import NavbarUserMenu from '../NavbarUserMenu';

// This creates a custom navbar item component that Docusaurus can recognize
// Wraps the NavbarUserMenu with AuthProvider to ensure useAuth hook works properly
const NavbarItemCustomNavbarUserMenu = (props) => {
  return (
    <AuthProvider>
      <NavbarUserMenu {...props} />
    </AuthProvider>
  );
};

export default NavbarItemCustomNavbarUserMenu;