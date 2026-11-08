/** an interaction hint, stacked above any earlier ones in the bottom-left corner */
export function createInfo(text: string): HTMLDivElement {
    let stack = document.querySelector<HTMLDivElement>('.info-stack');
    if (!stack) {
        stack = document.createElement('div');
        stack.className = 'info-stack';
        document.body.appendChild(stack);
    }
    const info = document.createElement('div');
    info.className = 'info';
    info.textContent = text;
    stack.appendChild(info);
    return info;
}
