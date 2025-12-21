import React from 'react';
import Navbar from '@theme-original/Navbar';
import NavbarAuth from '@site/src/components/NavbarAuth';

export default function NavbarWrapper(props) {
  return (
    <>
      <Navbar {...props} />
      <NavbarAuth />
    </>
  );
}
