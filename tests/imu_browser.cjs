/* Exercise the real HTTP/SSE page with the explicit synthetic USB test server.
   No physical serial device is opened by this test. */
const { chromium } = require('playwright');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
(async () => {
  const browser = await chromium.launch({ channel: 'msedge', headless: true });
  const context = await browser.newContext({ viewport: { width: 1440, height: 1000 } });
  await context.addInitScript(() => {
    window.drawnLabels = new Set();
    const original = CanvasRenderingContext2D.prototype.fillText;
    CanvasRenderingContext2D.prototype.fillText = function(text, ...args) {
      window.drawnLabels.add(text);
      return original.call(this, text, ...args);
    };
  });
  const page = await context.newPage(), errors = [];
  page.on('pageerror', e => errors.push(e.message));
  const url = process.env.IMU_URL || 'http://127.0.0.1:18774/';
  const output = path.resolve(__dirname, '../../tmp/h723_imu_local_20261011');
  fs.mkdirSync(output, { recursive: true });
  try {
    await page.goto(url);
    await page.getByRole('tab', { name: '陀螺仪 · H723', exact: true }).click();
    await page.waitForFunction(() => document.querySelector('#imu-port option[value="COM_TEST"]'));
    assert.equal(await page.locator('#imu-port').inputValue(), '', 'must not select a port silently');
    assert.equal(await page.locator('#imu-connect').isDisabled(), true);
    assert.equal(await page.locator('#imu-roll').textContent(), '—');
    await page.locator('#imu-port').selectOption('COM_TEST');
    await page.locator('#imu-connect').click();
    await page.waitForFunction(() => document.querySelector('#imu-state').dataset.state === 'streaming');
    assert.match(await page.locator('#imu-identity').textContent(), /STM32H723VGT6.*BMI088.*UID/);
    await page.waitForFunction(() => document.getElementById('imu-roll').textContent !== '—');
    await page.waitForTimeout(800);
    let state = await page.evaluate(async () => (await fetch('/api/imu/state')).json());
    assert.ok(state.history.length > 20);
    assert.equal(state.latest.valid, true);
    const labels = await page.evaluate(() => [...window.drawnLabels]);
    for (const text of ['时间 (s)', '角度 (°)', '加速度 (m/s²)', 'X 加速度 (m/s²)', 'Y 加速度 (m/s²)']) assert.ok(labels.includes(text), text);
    await page.screenshot({ path: path.join(output, 'h723-light.png'), fullPage: true });
    await page.locator('#imu-accel-mode').selectOption('accel_ms2');
    await page.waitForFunction(() => document.getElementById('imu-ax').textContent === '4.000');
    assert.equal(await page.locator('#imu-ay').textContent(), '5.000');
    await page.locator('#imu-accel-mode').selectOption('linear_accel_ms2');
    await page.locator('#btn-theme').click();
    await page.waitForTimeout(300); // Let the existing theme transition finish before visual QA.
    await page.screenshot({ path: path.join(output, 'h723-dark.png'), fullPage: true });
    await page.setViewportSize({ width: 430, height: 920 });
    await page.screenshot({ path: path.join(output, 'h723-mobile.png'), fullPage: true });
    const overflow = await page.evaluate(() => document.documentElement.scrollWidth > innerWidth);
    assert.equal(overflow, false, 'mobile content must not overflow sideways');
    await page.setViewportSize({ width: 1440, height: 1000 });
    const second = await context.newPage();
    await second.goto(url);
    await second.getByRole('tab', { name: '陀螺仪 · H723', exact: true }).click();
    await second.waitForFunction(() => {
      const badge = document.getElementById('role-badge');
      return !badge.classList.contains('hidden') && badge.textContent.includes('只读')
        && document.getElementById('imu-disconnect').disabled;
    });
    assert.equal(await second.locator('#imu-disconnect').isDisabled(), true);
    await second.close();
    await page.evaluate(() => fetch('/_fixture/silent', { method: 'POST', body: '{}' }));
    await page.waitForFunction(() => document.querySelector('#imu-state').dataset.state === 'stale');
    assert.equal(await page.locator('#imu-roll').textContent(), '—');
    state = await page.evaluate(async () => (await fetch('/api/imu/state')).json());
    assert.equal(state.latest.valid, false);
    assert.ok(state.history.length > 20, 'disconnect preserves history');
    await page.locator('#imu-disconnect').click();
    await page.waitForFunction(() => document.querySelector('#imu-state').dataset.state === 'offline');
    await page.evaluate(() => fetch('/_fixture/wrong-board', { method: 'POST', body: '{}' }));
    await page.locator('#imu-connect').click();
    await page.waitForFunction(() => document.querySelector('#imu-state').dataset.state === 'error');
    assert.equal(await page.locator('#imu-roll').textContent(), '—');
    assert.equal((await page.evaluate(async () => (await fetch('/api/imu/state')).json())).history.length, 0);
    assert.deepEqual(errors, []);
    console.log('H723 browser OK: explicit selection, exact handshake, plots/labels, accel modes, themes/mobile, readonly, stale data, wrong-board refusal.');
  } finally { await browser.close(); }
})().catch(e => { console.error(e); process.exitCode = 1; });
