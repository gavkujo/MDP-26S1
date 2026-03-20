export class CustomError extends Error {
	content;

	constructor(content) {
		super();
		this.content = content
	}

	msg(s) {
		this.message = s;
	}
}

export var methodType = {get : 'GET', post : 'POST', put : 'PUT', delete : 'DELETE'};

export default class BaseAPI {	
	static JSONRequest(api, method, headers, options, content) {
		// Always use the Next.js proxy to avoid CORS issues.
		// The proxy is configured in next.config.js to forward /api requests
		// to the backend server.
		const requestUrl = `/api${api}`;

		console.log(`🔗 Making request to: ${requestUrl}`);
		if (content) {
			console.log('📤 Request data:', content);
		}

		let requestOptions = {
			method: method,
			headers: {...headers, 'Content-Type': 'application/json'},
			...options
		}

		if (method === methodType.post || method === methodType.put) {
			requestOptions.body = JSON.stringify(content)
		}

		return new Promise((resolve, reject) => {
			fetch(requestUrl, requestOptions)
				.then(response => {
					console.log(`📥 Response status: ${response.status} ${response.statusText}`);
					
					if (!response.ok) {
						throw new CustomError(response);
					}

					response.json()
						.then(res => {
							console.log('✅ Response data:', res);
							if (res.error) {
								reject(JSON.stringify(res.error));
							}
							resolve(res.data);
						})
						.catch(err => {
							console.log('⚠️ JSON parse error, returning empty object');
							resolve({});
						});

				})
				.catch(async (err) => {
					console.log('❌ Request failed:', err);
					console.log(err)
					if (err instanceof CustomError) {

						// best effort to capture all cases of err handling
						let errStr = await err.content.json()
							.then(res => {
								if (res.errors) {
									return JSON.stringify(res.errors);
								}

								return "";
							}).catch(() => {
								return "";
							});

						err.msg(errStr);
						reject(err);

					} else {
						reject(err);
					}
				})
		})
	}

}
