/* Verify UI numbers -> wire commands -> pin telemetry using a SIM-only server.
   Never run this motion-producing fixture against a physical board. */
const {chromium} = require('playwright');
const assert = require('node:assert/strict');
const path = require('node:path');

(async () => {
  const browser = await chromium.launch({channel:'msedge',headless:true});
  const page = await browser.newPage({viewport:{width:1440,height:1000}});
  const errors = [], commands = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('request', request => {
    if (request.url().endsWith('/api/command')) commands.push(request.postDataJSON().body);
  });
  try {
    await page.goto(new URL('/debug', process.env.SERVO_TEST_URL || 'http://127.0.0.1:18779/').href);
    const snapshot = () => page.evaluate(async () => (await fetch('/api/state')).json());
    assert.equal((await snapshot()).demo, true, 'only exercise the simulated board');
    const s1 = page.locator('.servo-card[data-ch="1"]');
    const s2 = page.locator('.servo-card[data-ch="2"]');
    await page.waitForFunction(() =>
      document.querySelector('.servo-card[data-ch="1"] .angle').textContent === '120.0' &&
      document.querySelector('.servo-card[data-ch="2"] .angle').textContent === '0.0');
    assert.match(await s1.locator('h2').textContent(), /180°/);
    assert.match(await s2.locator('h2').textContent(), /270°/);
    assert.match(await s1.locator('.tag').textContent(), /PH9.*TIM12_CH2/);
    assert.match(await s2.locator('.tag').textContent(), /PE4.*TIM15_CH1N/);
    assert.equal(await s1.locator('.angle-input').inputValue(), '120');
    assert.equal(await s2.locator('.angle-input').inputValue(), '0');
    assert.deepEqual(commands, [], 'initial rendering must not send servo motion');
    await s1.locator('.servo-send').click();
    await s2.locator('.servo-send').click();
    await page.waitForFunction(() =>
      document.querySelector('.servo-card[data-ch="1"] .angle').textContent === '120.0' &&
      document.querySelector('.servo-card[data-ch="2"] .angle').textContent === '0.0');
    assert.equal(await s1.locator('.us').textContent(), '1833');
    assert.equal(await s2.locator('.us').textContent(), '500');
    assert.ok(commands.includes('SERVO 2 120'));
    assert.ok(commands.includes('SERVO 1 0'));
    assert.match((await snapshot()).servo.raw, /ch=2 angle=120.0 us=1833/);
    await s1.locator('.angle-input').fill('180.5');
    const before = commands.length;
    await s1.locator('.servo-send').click();
    await page.waitForFunction(() => document.getElementById('status').textContent.includes('180°'));
    assert.equal(commands.length, before, 'out-of-range PH9 commands stay local');
    await s2.locator('[data-deg="270"]').click();
    await page.waitForFunction(() => document.querySelector('.servo-card[data-ch="2"] .angle').textContent === '270.0');
    assert.equal(await s2.locator('.us').textContent(), '2500');
    await s1.locator('.angle-input').fill('120');
    await s2.locator('[data-deg="0"]').click();
    await page.waitForFunction(() => document.querySelector('.servo-card[data-ch="2"] .gauge-value').textContent === '0.0°');
    for (const width of [1440,390]) {
      await page.setViewportSize({width,height:1000});
      await page.waitForTimeout(250); // Allow gauge transitions and SVG paint to finish.
      assert.equal(await page.evaluate(() => document.documentElement.scrollWidth > innerWidth), false);
      await page.screenshot({path:path.join(__dirname,`../tmp/servo_mapping_${width}.png`),fullPage:true});
    }
    assert.deepEqual(errors, []);
    console.log('Servo browser OK: defaults, mapped commands/telemetry, S1 limit, S2 full travel, desktop/mobile layout.');
  } finally {
    await browser.close();
  }
})().catch(error => { console.error(error); process.exitCode=1; });
