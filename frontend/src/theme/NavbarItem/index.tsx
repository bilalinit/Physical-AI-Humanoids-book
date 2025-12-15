import React from 'react';
import OriginalNavbarItem from '@theme-original/NavbarItem';
import NavbarItemCustomNavbarUserMenu from './NavbarItemCustomNavbarUserMenu';

// This component wraps the original NavbarItem and adds support for our custom type
const NavbarItem = (props) => {
  const { type, ...restProps } = props;

  // If it's our custom type, render our component; otherwise use the original
  if (type === 'custom-NavbarUserMenu') {
    return <NavbarItemCustomNavbarUserMenu {...props} />;
  }

  // For all other types, use the original NavbarItem
  return <OriginalNavbarItem {...props} />;
};

export default NavbarItem;