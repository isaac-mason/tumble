import { dashboard, type Panel } from 'dashcat';

/**
 * an example's panel, pinned to the top-right corner. dashcat places panels by left and top and writes an explicit left
 * when it clamps them inside a resized window, so the pin is applied again after its own resize handler
 */
export function createPanel(title: string): Panel {
    const panel = dashboard().panel({ title });
    const pin = () => {
        panel.root.style.left = 'auto';
        panel.root.style.right = '16px';
        panel.root.style.top = '16px';
    };
    pin();
    window.addEventListener('resize', pin);
    return panel;
}
