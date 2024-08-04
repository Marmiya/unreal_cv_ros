import unrealcv_api

client = unrealcv_api.UnrealCv_API(9000, '127.0.0.1', (1024, 768))


status = client.client.request('vget /unrealcv/status')

print(status)