import React, { useState, useEffect, useRef } from 'react';
import { useAuth } from '@site/src/hooks/useAuth';
import Link from '@docusaurus/Link';
import styles from './NavbarUserMenu.module.css';

const NavbarUserMenu: React.FC = () => {
  const { user, loading } = useAuth();
  const [isDropdownOpen, setIsDropdownOpen] = useState(false);
  const dropdownRef = useRef<HTMLDivElement>(null);

  // Close dropdown when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setIsDropdownOpen(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  const handleSignOut = () => {
    localStorage.removeItem('auth_session');
    localStorage.removeItem('chatkit-thread-id');
    window.location.href = '/signin';
  };

  const handleToggleDropdown = () => {
    setIsDropdownOpen(!isDropdownOpen);
  };

  // Get first letter of user's name for the icon
  const getInitials = () => {
    if (user?.name) {
      return user.name.charAt(0).toUpperCase();
    }
    return user?.email?.charAt(0).toUpperCase() || 'U';
  };

  if (loading) {
    return <div className={styles.container}>Loading...</div>;
  }

  if (!user) {
    // Show sign in link when not authenticated
    return (
      <div className={styles.container}>
        <Link to="/signin" className={styles.signInLink}>
          Sign In
        </Link>
      </div>
    );
  }

  // Show user menu when authenticated
  return (
    <div className={styles.container} ref={dropdownRef}>
      <button
        className={styles.dropdownTrigger}
        onClick={handleToggleDropdown}
        aria-expanded={isDropdownOpen}
        aria-haspopup="true"
      >
        <span className={styles.userIcon} aria-label="User icon">
          {getInitials()}
        </span>
        <span className={styles.userName}>{user.name || user.email}</span>
      </button>

      {isDropdownOpen && (
        <ul className={styles.dropdownMenu} role="menu">
          <li className={styles.dropdownItem} role="none">
            <Link to="/profile" role="menuitem" onClick={() => setIsDropdownOpen(false)}>
              Profile
            </Link>
          </li>
          <li className={styles.dropdownItem} role="none">
            <button
              onClick={handleSignOut}
              role="menuitem"
              className={styles.signOutButton}
            >
              Sign Out
            </button>
          </li>
        </ul>
      )}
    </div>
  );
};

export default NavbarUserMenu;