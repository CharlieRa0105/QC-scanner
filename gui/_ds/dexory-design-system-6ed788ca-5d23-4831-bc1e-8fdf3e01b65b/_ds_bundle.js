/* @ds-bundle: {"format":3,"namespace":"DexoryDesignSystem_6ed788","components":[],"sourceHashes":{"ui_kits/dexoryview/DataTable.jsx":"f903ef0bbd19","ui_kits/dexoryview/Primitives.jsx":"b1d17623e100","ui_kits/dexoryview/Sidebar.jsx":"d6ce67c470e6","ui_kits/dexoryview/Topbar.jsx":"3fcf19d01349"},"inlinedExternals":[],"unexposedExports":[]} */

(() => {

const __ds_ns = (window.DexoryDesignSystem_6ed788 = window.DexoryDesignSystem_6ed788 || {});

const __ds_scope = {};

(__ds_ns.__errors = __ds_ns.__errors || []);

// ui_kits/dexoryview/DataTable.jsx
try { (() => {
// DataTable.jsx — sortable data table with mono numerics
function DataTable({
  columns,
  rows,
  onRowClick
}) {
  const styles = {
    table: {
      width: "100%",
      borderCollapse: "collapse",
      fontFamily: "var(--font-sans)"
    },
    th: {
      textAlign: "left",
      padding: "10px 16px",
      fontSize: 12,
      fontWeight: 700,
      color: "var(--color-charcoal-800)",
      borderBottom: "1px solid var(--color-charcoal-100)",
      background: "var(--color-charcoal-50)",
      whiteSpace: "nowrap"
    },
    td: {
      padding: "12px 16px",
      fontSize: 14,
      fontWeight: 500,
      borderBottom: "1px solid var(--color-charcoal-100)",
      verticalAlign: "middle"
    },
    tr: clickable => ({
      cursor: clickable ? "pointer" : "default",
      background: "#fff",
      transition: "background 100ms"
    })
  };
  return /*#__PURE__*/React.createElement("table", {
    style: styles.table
  }, /*#__PURE__*/React.createElement("thead", null, /*#__PURE__*/React.createElement("tr", null, columns.map(c => /*#__PURE__*/React.createElement("th", {
    key: c.key,
    style: {
      ...styles.th,
      textAlign: c.align || "left",
      width: c.width
    }
  }, /*#__PURE__*/React.createElement("span", {
    style: {
      display: "inline-flex",
      alignItems: "center",
      gap: 4
    }
  }, c.label, c.sortable && /*#__PURE__*/React.createElement(Icon, {
    name: "chevron-selector",
    size: 12
  })))))), /*#__PURE__*/React.createElement("tbody", null, rows.map((r, i) => /*#__PURE__*/React.createElement("tr", {
    key: i,
    style: styles.tr(!!onRowClick),
    onClick: () => onRowClick && onRowClick(r),
    onMouseEnter: e => e.currentTarget.style.background = "var(--color-charcoal-50)",
    onMouseLeave: e => e.currentTarget.style.background = "#fff"
  }, columns.map(c => /*#__PURE__*/React.createElement("td", {
    key: c.key,
    style: {
      ...styles.td,
      textAlign: c.align || "left",
      fontFamily: c.mono ? "var(--font-mono)" : "var(--font-sans)",
      fontFeatureSettings: c.mono ? "\"tnum\"" : undefined,
      color: c.muted ? "var(--color-charcoal-600)" : "inherit"
    }
  }, c.render ? c.render(r[c.key], r) : r[c.key]))))));
}
Object.assign(window, {
  DataTable
});
})(); } catch (e) { __ds_ns.__errors.push({ path: "ui_kits/dexoryview/DataTable.jsx", error: String((e && e.message) || e) }); }

// ui_kits/dexoryview/Primitives.jsx
try { (() => {
// Primitives.jsx — Button, Badge, Card, Modal, Toast
function Button({
  variant = "primary",
  size = "md",
  iconLeft,
  iconRight,
  iconOnly,
  children,
  onClick,
  disabled,
  ariaLabel
}) {
  // Variant styles: flat palette, pill radius, 500-weight label.
  const palette = {
    primary: {
      bg: "#000",
      fg: "#fff",
      hover: "#2F2F2D",
      border: "transparent"
    },
    accent: {
      bg: "#5631EA",
      fg: "#fff",
      hover: "#4825C4",
      border: "transparent"
    },
    secondary: {
      bg: "#F6F7F4",
      fg: "#000",
      hover: "#ECEDEA",
      border: "transparent"
    },
    tertiary: {
      bg: "transparent",
      fg: "#000",
      hover: "#F6F7F4",
      border: "#000"
    },
    ghost: {
      bg: "transparent",
      fg: "#000",
      hover: "#ECEDEA",
      border: "transparent"
    },
    link: {
      bg: "transparent",
      fg: "#5631EA",
      hover: "#4825C4",
      border: "transparent"
    }
  }[variant];

  // Size ramp — 4/8 grid, +8px per step
  const sz = {
    xs: {
      h: 24,
      px: 10,
      f: 11,
      i: 12,
      gap: 4
    },
    sm: {
      h: 32,
      px: 14,
      f: 13,
      i: 14,
      gap: 6
    },
    md: {
      h: 40,
      px: 18,
      f: 14,
      i: 16,
      gap: 8
    },
    lg: {
      h: 48,
      px: 22,
      f: 15,
      i: 18,
      gap: 10
    }
  }[size];
  const isLink = variant === "link";
  const onlyIcon = !!iconOnly && !children;
  const [hover, setHover] = React.useState(false);
  return /*#__PURE__*/React.createElement("button", {
    disabled: disabled,
    onClick: onClick,
    "aria-label": ariaLabel || (onlyIcon ? iconOnly : undefined),
    onMouseEnter: () => setHover(true),
    onMouseLeave: () => setHover(false),
    style: {
      height: isLink ? "auto" : sz.h,
      padding: isLink ? 0 : onlyIcon ? 0 : `0 ${sz.px}px`,
      width: onlyIcon ? sz.h : undefined,
      borderRadius: isLink ? 0 : 999,
      fontFamily: "var(--font-sans)",
      fontWeight: 500,
      fontSize: sz.f,
      lineHeight: 1,
      background: hover && !disabled ? palette.hover : palette.bg,
      color: isLink && hover ? palette.hover : palette.fg,
      border: `1px solid ${palette.border}`,
      cursor: disabled ? "default" : "pointer",
      display: "inline-flex",
      alignItems: "center",
      justifyContent: "center",
      gap: sz.gap,
      opacity: disabled ? 0.4 : 1,
      transition: "background 150ms ease-out, color 150ms ease-out, border-color 150ms ease-out",
      whiteSpace: "nowrap",
      textDecoration: isLink && hover ? "underline" : "none",
      textUnderlineOffset: 3
    }
  }, iconLeft && /*#__PURE__*/React.createElement(Icon, {
    name: iconLeft,
    size: sz.i
  }), onlyIcon ? /*#__PURE__*/React.createElement(Icon, {
    name: iconOnly,
    size: sz.i
  }) : children, iconRight && /*#__PURE__*/React.createElement(Icon, {
    name: iconRight,
    size: sz.i
  }));
}
function Badge({
  type = "neutral",
  children,
  dot = true
}) {
  const palette = {
    error: {
      bg: "#FFF1F1",
      fg: "#A10000"
    },
    warning: {
      bg: "#FEF3C6",
      fg: "#7B3306"
    },
    success: {
      bg: "#E7F5E8",
      fg: "#125D1A"
    },
    info: {
      bg: "#EEE7FC",
      fg: "#5631EA"
    },
    neutral: {
      bg: "#F6F7F4",
      fg: "#000"
    }
  }[type];
  return /*#__PURE__*/React.createElement("span", {
    style: {
      display: "inline-flex",
      alignItems: "center",
      gap: 6,
      background: palette.bg,
      color: palette.fg,
      padding: "2px 8px",
      borderRadius: 4,
      fontFamily: "var(--font-sans)",
      fontWeight: 700,
      fontSize: 12,
      lineHeight: "16px"
    }
  }, dot && /*#__PURE__*/React.createElement("span", {
    style: {
      width: 6,
      height: 6,
      borderRadius: "50%",
      background: palette.fg
    }
  }), children);
}
function Card({
  title,
  actions,
  children,
  padded = true
}) {
  return /*#__PURE__*/React.createElement("section", {
    style: {
      background: "#fff",
      border: "1px solid var(--color-charcoal-100)",
      borderRadius: 0,
      fontFamily: "var(--font-sans)"
    }
  }, title && /*#__PURE__*/React.createElement("div", {
    style: {
      padding: "16px 20px",
      borderBottom: "1px solid var(--color-charcoal-100)",
      display: "flex",
      alignItems: "center"
    }
  }, /*#__PURE__*/React.createElement("div", {
    style: {
      fontWeight: 700,
      fontSize: 16,
      flex: 1
    }
  }, title), actions), /*#__PURE__*/React.createElement("div", {
    style: {
      padding: padded ? 20 : 0
    }
  }, children));
}
function Modal({
  open,
  title,
  children,
  onClose,
  footer
}) {
  if (!open) return null;
  return /*#__PURE__*/React.createElement("div", {
    style: {
      position: "fixed",
      inset: 0,
      background: "rgba(16,24,40,.5)",
      zIndex: 50,
      display: "flex",
      alignItems: "center",
      justifyContent: "center",
      fontFamily: "var(--font-sans)"
    },
    onClick: onClose
  }, /*#__PURE__*/React.createElement("div", {
    onClick: e => e.stopPropagation(),
    style: {
      background: "#fff",
      width: 520,
      boxShadow: "0 8px 32px rgba(0,0,0,.16)",
      borderRadius: 0
    }
  }, /*#__PURE__*/React.createElement("div", {
    style: {
      padding: "20px 24px",
      borderBottom: "1px solid var(--color-charcoal-100)",
      display: "flex",
      alignItems: "center"
    }
  }, /*#__PURE__*/React.createElement("div", {
    style: {
      fontWeight: 700,
      fontSize: 20,
      flex: 1
    }
  }, title), /*#__PURE__*/React.createElement("button", {
    style: {
      border: 0,
      background: "transparent",
      cursor: "pointer",
      padding: 4
    },
    onClick: onClose
  }, /*#__PURE__*/React.createElement(Icon, {
    name: "x-circle",
    size: 18
  }))), /*#__PURE__*/React.createElement("div", {
    style: {
      padding: 24,
      fontSize: 14,
      fontWeight: 500,
      color: "var(--color-charcoal-700)",
      lineHeight: "20px"
    }
  }, children), /*#__PURE__*/React.createElement("div", {
    style: {
      padding: "16px 24px",
      borderTop: "1px solid var(--color-charcoal-100)",
      display: "flex",
      justifyContent: "flex-end",
      gap: 8
    }
  }, footer)));
}
function Toast({
  type = "success",
  message,
  onClose
}) {
  if (!message) return null;
  const icon = {
    success: "check-circle",
    error: "x-circle",
    warning: "alert-triangle",
    info: "info-circle"
  }[type];
  const fg = {
    success: "#125D1A",
    error: "#A10000",
    warning: "#7B3306",
    info: "#5631EA"
  }[type];
  return /*#__PURE__*/React.createElement("div", {
    style: {
      position: "fixed",
      left: 24,
      bottom: 24,
      background: "#fff",
      border: "1px solid var(--color-charcoal-100)",
      boxShadow: "0 4px 16px rgba(0,0,0,.12)",
      padding: "12px 16px",
      display: "flex",
      alignItems: "center",
      gap: 10,
      fontFamily: "var(--font-sans)",
      fontSize: 14,
      fontWeight: 500,
      minWidth: 280
    }
  }, /*#__PURE__*/React.createElement("span", {
    style: {
      color: fg
    }
  }, /*#__PURE__*/React.createElement(Icon, {
    name: icon,
    size: 18
  })), /*#__PURE__*/React.createElement("span", {
    style: {
      flex: 1
    }
  }, message), /*#__PURE__*/React.createElement("button", {
    onClick: onClose,
    style: {
      border: 0,
      background: "transparent",
      cursor: "pointer",
      color: "var(--color-charcoal-600)"
    }
  }, "\u2715"));
}
Object.assign(window, {
  Button,
  Badge,
  Card,
  Modal,
  Toast
});
})(); } catch (e) { __ds_ns.__errors.push({ path: "ui_kits/dexoryview/Primitives.jsx", error: String((e && e.message) || e) }); }

// ui_kits/dexoryview/Sidebar.jsx
try { (() => {
// Sidebar.jsx — DexoryView primary nav (collapsible icon rail)
function Icon({
  name,
  size = 18,
  stroke = 2
}) {
  return /*#__PURE__*/React.createElement("svg", {
    width: size,
    height: size,
    strokeWidth: stroke,
    style: {
      flexShrink: 0
    }
  }, /*#__PURE__*/React.createElement("use", {
    href: "../../assets/icons.svg#" + name
  }));
}
function Sidebar({
  current,
  onNavigate
}) {
  const [collapsed, setCollapsed] = React.useState(true);
  const nav = [{
    id: "performance",
    icon: "bar-chart",
    label: "Overview"
  }, {
    id: "counts",
    icon: "cube",
    label: "Integrity"
  }, {
    id: "robots",
    icon: "atom",
    label: "Optimise",
    pulse: true
  }, {
    id: "reports",
    icon: "bar-chart",
    label: "Trends"
  }, {
    id: "tasks",
    icon: "clipboard",
    label: "Tasks",
    count: 7
  }];
  const bottomNav = [{
    id: "notifs",
    icon: "bell"
  }, {
    id: "devices",
    icon: "cube"
  }, {
    id: "settings",
    icon: "refresh"
  }, {
    id: "home",
    icon: "layers"
  }];

  // ── COLLAPSED: vertical icon rail ───────────────────────────────────
  const railStyles = {
    nav: {
      width: 80,
      minHeight: "100vh",
      background: "var(--color-charcoal-900)",
      color: "#fff",
      padding: "10px 0",
      display: "flex",
      flexDirection: "column",
      alignItems: "center",
      gap: 2,
      fontFamily: "var(--font-sans)",
      position: "sticky",
      top: 0,
      flexShrink: 0,
      boxSizing: "border-box"
    },
    toggle: {
      width: 40,
      height: 40,
      border: 0,
      background: "transparent",
      color: "rgba(255,255,255,.85)",
      cursor: "pointer",
      borderRadius: 6,
      display: "inline-flex",
      alignItems: "center",
      justifyContent: "center",
      margin: "4px 0 2px"
    },
    brand: {
      width: 44,
      height: 44,
      borderRadius: 8,
      background: "var(--color-lime-500)",
      display: "inline-flex",
      alignItems: "center",
      justifyContent: "center",
      margin: "6px 0 4px"
    },
    iconOnly: active => ({
      width: 40,
      height: 40,
      borderRadius: 6,
      border: 0,
      cursor: "pointer",
      background: active ? "#1B2230" : "transparent",
      color: active ? "#fff" : "rgba(255,255,255,.75)",
      display: "inline-flex",
      alignItems: "center",
      justifyContent: "center",
      position: "relative"
    }),
    divider: {
      width: 32,
      height: 1,
      background: "rgba(255,255,255,.10)",
      margin: "10px 0 8px"
    },
    item: active => ({
      width: 64,
      padding: "8px 4px 10px",
      borderRadius: 8,
      background: active ? "#1B2230" : "transparent",
      color: active ? "#fff" : "rgba(255,255,255,.75)",
      display: "flex",
      flexDirection: "column",
      alignItems: "center",
      gap: 6,
      cursor: "pointer",
      position: "relative",
      transition: "background 120ms ease-out, color 120ms ease-out"
    }),
    iconWrap: {
      position: "relative",
      display: "inline-flex"
    },
    label: {
      fontSize: 12,
      fontWeight: 600,
      lineHeight: 1.1,
      textAlign: "center",
      letterSpacing: "-0.005em"
    },
    dot: {
      position: "absolute",
      top: -2,
      right: -4,
      width: 6,
      height: 6,
      borderRadius: "50%",
      background: "var(--color-lime-500)"
    },
    count: {
      position: "absolute",
      top: -4,
      right: -8,
      minWidth: 16,
      height: 16,
      padding: "0 4px",
      borderRadius: 8,
      background: "var(--color-lime-500)",
      color: "#000",
      fontSize: 10,
      fontWeight: 700,
      fontFamily: "var(--font-mono)",
      display: "inline-flex",
      alignItems: "center",
      justifyContent: "center"
    },
    bottom: {
      marginTop: "auto",
      display: "flex",
      flexDirection: "column",
      alignItems: "center",
      gap: 2,
      paddingBottom: 10
    }
  };
  if (collapsed) {
    return /*#__PURE__*/React.createElement("nav", {
      style: railStyles.nav
    }, /*#__PURE__*/React.createElement("button", {
      style: railStyles.toggle,
      onClick: () => setCollapsed(false),
      title: "Expand sidebar",
      "aria-label": "Expand sidebar"
    }, /*#__PURE__*/React.createElement(Icon, {
      name: "sidebar",
      size: 18
    })), /*#__PURE__*/React.createElement("div", {
      style: railStyles.brand
    }, /*#__PURE__*/React.createElement("img", {
      src: "../../assets/logomark.svg",
      alt: "Dexory",
      style: {
        width: 22,
        height: 22
      }
    })), /*#__PURE__*/React.createElement("button", {
      style: railStyles.iconOnly(false),
      title: "Notifications",
      "aria-label": "Notifications"
    }, /*#__PURE__*/React.createElement(Icon, {
      name: "bell",
      size: 18
    })), /*#__PURE__*/React.createElement("div", {
      style: railStyles.divider
    }), nav.map(it => {
      const active = current === it.id;
      return /*#__PURE__*/React.createElement("div", {
        key: it.id,
        style: railStyles.item(active),
        onClick: () => onNavigate && onNavigate(it.id),
        onMouseEnter: e => {
          if (!active) e.currentTarget.style.background = "rgba(255,255,255,.05)";
        },
        onMouseLeave: e => {
          if (!active) e.currentTarget.style.background = "transparent";
        }
      }, /*#__PURE__*/React.createElement("span", {
        style: railStyles.iconWrap
      }, /*#__PURE__*/React.createElement(Icon, {
        name: it.icon,
        size: 20
      }), it.pulse && /*#__PURE__*/React.createElement("span", {
        style: railStyles.dot
      }), it.count != null && /*#__PURE__*/React.createElement("span", {
        style: railStyles.count
      }, it.count)), /*#__PURE__*/React.createElement("span", {
        style: railStyles.label
      }, it.label));
    }), /*#__PURE__*/React.createElement("div", {
      style: railStyles.bottom
    }, bottomNav.map(it => /*#__PURE__*/React.createElement("button", {
      key: it.id,
      style: railStyles.iconOnly(false),
      title: it.id,
      "aria-label": it.id
    }, /*#__PURE__*/React.createElement(Icon, {
      name: it.icon,
      size: 18
    })))));
  }

  // ── EXPANDED: rail + full labeled panel ──────────────────────────────
  const expStyles = {
    wrap: {
      display: "flex",
      fontFamily: "var(--font-sans)",
      position: "sticky",
      top: 0,
      minHeight: "100vh"
    },
    panel: {
      width: 220,
      background: "var(--color-charcoal-900)",
      color: "#fff",
      padding: "18px 12px",
      boxSizing: "border-box",
      display: "flex",
      flexDirection: "column",
      gap: 2,
      borderLeft: "1px solid rgba(255,255,255,.06)"
    },
    brandRow: {
      display: "flex",
      alignItems: "center",
      gap: 10,
      padding: "4px 8px 16px"
    },
    brandLogo: {
      width: 28,
      height: 28,
      borderRadius: 6
    },
    brandName: {
      fontWeight: 700,
      fontSize: 16,
      letterSpacing: "-0.01em"
    },
    section: {
      fontSize: 11,
      letterSpacing: "0.08em",
      textTransform: "uppercase",
      color: "rgba(255,255,255,.45)",
      padding: "12px 10px 6px",
      fontWeight: 700
    },
    row: active => ({
      display: "flex",
      alignItems: "center",
      gap: 12,
      padding: "8px 10px",
      borderRadius: 8,
      color: active ? "#fff" : "rgba(255,255,255,.8)",
      background: active ? "rgba(255,255,255,.08)" : "transparent",
      fontSize: 14,
      fontWeight: 500,
      cursor: "pointer"
    }),
    count: {
      marginLeft: "auto",
      fontSize: 11,
      fontWeight: 700,
      fontFamily: "var(--font-mono)",
      color: "rgba(255,255,255,.7)"
    },
    dot: {
      width: 6,
      height: 6,
      borderRadius: "50%",
      background: "var(--color-lime-500)",
      marginLeft: "auto"
    },
    footer: {
      marginTop: "auto",
      padding: "12px 8px",
      display: "flex",
      alignItems: "center",
      gap: 10
    },
    avatar: {
      width: 28,
      height: 28,
      borderRadius: "50%",
      background: "var(--color-lime-500)",
      color: "#000",
      display: "flex",
      alignItems: "center",
      justifyContent: "center",
      fontWeight: 700,
      fontSize: 12
    }
  };
  return /*#__PURE__*/React.createElement("div", {
    style: expStyles.wrap
  }, /*#__PURE__*/React.createElement("nav", {
    style: railStyles.nav
  }, /*#__PURE__*/React.createElement("button", {
    style: railStyles.toggle,
    onClick: () => setCollapsed(true),
    title: "Collapse sidebar",
    "aria-label": "Collapse sidebar"
  }, /*#__PURE__*/React.createElement(Icon, {
    name: "sidebar",
    size: 18
  })), /*#__PURE__*/React.createElement("div", {
    style: railStyles.brand
  }, /*#__PURE__*/React.createElement("img", {
    src: "../../assets/logomark.svg",
    alt: "Dexory",
    style: {
      width: 22,
      height: 22
    }
  })), /*#__PURE__*/React.createElement("button", {
    style: railStyles.iconOnly(false),
    title: "Notifications",
    "aria-label": "Notifications"
  }, /*#__PURE__*/React.createElement(Icon, {
    name: "bell",
    size: 18
  })), /*#__PURE__*/React.createElement("div", {
    style: railStyles.divider
  }), nav.map(it => {
    const active = current === it.id;
    return /*#__PURE__*/React.createElement("div", {
      key: it.id,
      style: railStyles.item(active),
      onClick: () => onNavigate && onNavigate(it.id)
    }, /*#__PURE__*/React.createElement(Icon, {
      name: it.icon,
      size: 18
    }), /*#__PURE__*/React.createElement("span", {
      style: railStyles.label
    }, it.label), it.pulse && /*#__PURE__*/React.createElement("span", {
      style: railStyles.dot
    }));
  }), /*#__PURE__*/React.createElement("div", {
    style: railStyles.bottom
  }, bottomNav.map(it => /*#__PURE__*/React.createElement("button", {
    key: it.id,
    style: railStyles.iconOnly(false),
    "aria-label": it.id
  }, /*#__PURE__*/React.createElement(Icon, {
    name: it.icon,
    size: 18
  }))))), /*#__PURE__*/React.createElement("aside", {
    style: expStyles.panel
  }, /*#__PURE__*/React.createElement("div", {
    style: expStyles.brandRow
  }, /*#__PURE__*/React.createElement("img", {
    src: "../../assets/logomark.svg",
    alt: "",
    style: expStyles.brandLogo
  }), /*#__PURE__*/React.createElement("span", {
    style: expStyles.brandName
  }, "DexoryView")), /*#__PURE__*/React.createElement("div", {
    style: expStyles.section
  }, "Integrity module"), nav.map(it => {
    const active = current === it.id;
    return /*#__PURE__*/React.createElement("div", {
      key: it.id,
      style: expStyles.row(active),
      onClick: () => onNavigate && onNavigate(it.id)
    }, /*#__PURE__*/React.createElement(Icon, {
      name: it.icon,
      size: 18
    }), /*#__PURE__*/React.createElement("span", null, it.label), it.count != null && /*#__PURE__*/React.createElement("span", {
      style: expStyles.count
    }, it.count), it.pulse && /*#__PURE__*/React.createElement("span", {
      style: expStyles.dot
    }));
  }), /*#__PURE__*/React.createElement("div", {
    style: expStyles.footer
  }, /*#__PURE__*/React.createElement("div", {
    style: expStyles.avatar
  }, "EA"), /*#__PURE__*/React.createElement("div", {
    style: {
      fontSize: 13,
      fontWeight: 500
    }
  }, "Eli Adebayo", /*#__PURE__*/React.createElement("div", {
    style: {
      fontSize: 11,
      color: "rgba(255,255,255,.5)",
      marginTop: 2
    }
  }, "Supervisor \xB7 DC3")))));
}
Object.assign(window, {
  Sidebar,
  Icon
});
})(); } catch (e) { __ds_ns.__errors.push({ path: "ui_kits/dexoryview/Sidebar.jsx", error: String((e && e.message) || e) }); }

// ui_kits/dexoryview/Topbar.jsx
try { (() => {
// Topbar.jsx — page header + breadcrumbs + search + bell + avatar
function Topbar({
  title,
  breadcrumbs = [],
  actions = null
}) {
  const styles = {
    wrap: {
      display: "flex",
      flexDirection: "column",
      gap: 0,
      padding: "20px 32px 16px",
      background: "#fff",
      borderBottom: "1px solid var(--color-charcoal-100)",
      fontFamily: "var(--font-sans)"
    },
    crumb: {
      fontSize: 12,
      color: "var(--color-charcoal-600)",
      fontWeight: 500
    },
    crumbSep: {
      margin: "0 6px",
      color: "var(--color-charcoal-400)"
    },
    row: {
      display: "flex",
      alignItems: "center",
      gap: 16,
      marginTop: 8
    },
    h1: {
      fontSize: 28,
      fontWeight: 700,
      letterSpacing: "-0.02em",
      margin: 0,
      flex: 1
    },
    search: {
      display: "flex",
      alignItems: "center",
      gap: 8,
      background: "#F6F7F4",
      border: "1px solid transparent",
      borderRadius: 6,
      padding: "0 12px",
      height: 36,
      width: 280
    },
    searchIn: {
      border: 0,
      background: "transparent",
      outline: "none",
      fontFamily: "var(--font-sans)",
      fontSize: 14,
      fontWeight: 500,
      flex: 1
    },
    iconBtn: {
      width: 36,
      height: 36,
      border: "1px solid var(--color-charcoal-100)",
      background: "#fff",
      borderRadius: 6,
      display: "flex",
      alignItems: "center",
      justifyContent: "center",
      cursor: "pointer",
      position: "relative"
    },
    dot: {
      position: "absolute",
      top: 8,
      right: 8,
      width: 8,
      height: 8,
      borderRadius: "50%",
      background: "#DA1E28",
      border: "2px solid #fff"
    },
    avatar: {
      width: 32,
      height: 32,
      borderRadius: "50%",
      background: "var(--color-lime-500)",
      color: "#000",
      display: "flex",
      alignItems: "center",
      justifyContent: "center",
      fontWeight: 700,
      fontSize: 12
    }
  };
  return /*#__PURE__*/React.createElement("header", {
    style: styles.wrap
  }, /*#__PURE__*/React.createElement("div", {
    style: styles.crumb
  }, breadcrumbs.map((c, i) => /*#__PURE__*/React.createElement(React.Fragment, {
    key: i
  }, i > 0 && /*#__PURE__*/React.createElement("span", {
    style: styles.crumbSep
  }, "/"), /*#__PURE__*/React.createElement("span", null, c)))), /*#__PURE__*/React.createElement("div", {
    style: styles.row
  }, /*#__PURE__*/React.createElement("h1", {
    style: styles.h1
  }, title), /*#__PURE__*/React.createElement("div", {
    style: styles.search
  }, /*#__PURE__*/React.createElement(Icon, {
    name: "search",
    size: 16
  }), /*#__PURE__*/React.createElement("input", {
    style: styles.searchIn,
    placeholder: "Search locations, tasks, SKUs\u2026"
  })), /*#__PURE__*/React.createElement("button", {
    style: styles.iconBtn
  }, /*#__PURE__*/React.createElement(Icon, {
    name: "bell",
    size: 18
  }), /*#__PURE__*/React.createElement("span", {
    style: styles.dot
  })), actions, /*#__PURE__*/React.createElement("div", {
    style: styles.avatar
  }, "EA")));
}
Object.assign(window, {
  Topbar
});
})(); } catch (e) { __ds_ns.__errors.push({ path: "ui_kits/dexoryview/Topbar.jsx", error: String((e && e.message) || e) }); }

})();
