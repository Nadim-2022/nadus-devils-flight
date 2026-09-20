# Nadus Devil's Flight
Nadus Devil's Flight (NDF) is an open-source, high-performance drone flight controller.

## DeepSeek API key
Create a local environment file from the template and add your real key before running any DeepSeek-dependent tooling:

```bash
cp .env.example .env
# edit .env and replace the placeholder value for DEEPSEEK_API_KEY
```

The template stores the key as:

```env
DEEPSEEK_API_KEY=your_deepseek_api_key_here
DEEPSEEK_BASE_URL=https://api.deepseek.com
DEEPSEEK_MODEL=deepseek-chat
```

Do not commit the real `.env` file; it is ignored by git.

# References
https://deepwiki.com/ExpressLRS/ExpressLRS/3.1-crsf-protocol-and-router
https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md#single-wire-half-duplex-uart